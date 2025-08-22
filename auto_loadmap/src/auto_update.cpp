#include "auto_loadmap/auto_update.h"

AutoLoadMapSystem::AutoLoadMapSystem(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh), tf_buffer_(), tf_listener_(tf_buffer_),
      added_points_cloud(new pcl::PointCloud<pcl::PointXYZI>),
      removed_points_cloud(new pcl::PointCloud<pcl::PointXYZI>),
      current_merged_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
      should_stop_(false)
{
    private_nh_.param<std::string>("map_config_path", map_config_path_,
                                   "/workspaces/fusion_ekf/Localization_Indoor/src/auto_loadmap/map/config");
    private_nh_.param<std::string>("base_pcd_path", base_pcd_path_,
                                   "/workspaces/fusion_ekf/Localization_Indoor/src/auto_loadmap/map/config");

    ROS_INFO("[AutoLoadMapSystem] Node initialized.");
    bool check_boundaries = checkBoundariesFile(base_pcd_path_);
    if (check_boundaries)
    {
        ROS_INFO("[checkBoundariesFile] Checked boundaries for all areas.");
    }
    setupPublishers();
    setupSubscribers();
    setupServices();
    startMapUpdateThread();
}

AutoLoadMapSystem::~AutoLoadMapSystem()
{
    should_stop_ = true;
    map_update_cv_.notify_all();
    if (map_update_thread_.joinable())
    {
        map_update_thread_.join();
    }
}

/**
 *
 * CHECK & CREATE BOUNDARIES FILE
 *
 * **/
bool AutoLoadMapSystem::checkBoundariesFile(const std::string &path_to_pcd)
{
    ROS_INFO("[AutoLoadMapSystem] Checking boundaries for all areas.");
    for (const auto &subdir : fs::directory_iterator(path_to_pcd))
    {
        if (fs::is_directory(subdir))
        {
            std::string subdir_path = subdir.path().string();
            std::string yaml_path = subdir_path + "/map_boundaries.yaml";
            std::cout << "Checking path: " << yaml_path << std::endl;
            if (!fs::exists(yaml_path))
            {
                generateYamlForArea(path_to_pcd);
            }
            std::cout << "Checked: " << yaml_path << std::endl;
        }
    }
    return true;
}

void AutoLoadMapSystem::generateYamlForArea(const std::string &input_pcd)
{
    for (const auto &subdir : fs::directory_iterator(input_pcd))
    {
        if (fs::is_directory(subdir)) // ha dong, thanh xuan
        {
            std::string subdir_name = subdir.path().filename().string();
            std::string subdir_path = subdir.path().string();
            std::string pcd_path = subdir_path + "/pcd";
            std::string yaml_path = subdir_path + "/map_boundaries.yaml";

            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "files" << YAML::Value << YAML::BeginSeq;

            for (const auto &entry : fs::directory_iterator(pcd_path))
            {
                if (entry.path().extension() == ".pcd")
                {
                    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

                    if (pcl::io::loadPCDFile<PointT>(entry.path().string(), *cloud) == -1)
                    {
                        ROS_ERROR("[MapAutoLoad] Failed to load PCD file: %s",
                                  entry.path().string().c_str());
                        continue;
                    }

                    PointT min_pt, max_pt;
                    pcl::getMinMax3D(*cloud, min_pt, max_pt);

                    out << YAML::BeginMap;
                    out << YAML::Key << "file" << YAML::Value
                        << entry.path().filename().string();
                    out << YAML::Key << "min" << YAML::Value << YAML::Flow
                        << YAML::BeginSeq << min_pt.x << min_pt.y << min_pt.z
                        << YAML::EndSeq;
                    out << YAML::Key << "max" << YAML::Value << YAML::Flow
                        << YAML::BeginSeq << max_pt.x << max_pt.y << max_pt.z
                        << YAML::EndSeq;
                    out << YAML::EndMap;
                    ROS_INFO("[generateYamlForArea] Created successfully %s boundaries map",
                             entry.path().filename().string().c_str());
                }
                else
                {
                    ROS_INFO("[generateYamlForArea] No .pcd file found in %s",
                             pcd_path.c_str());
                }
            }

            out << YAML::EndSeq;
            out << YAML::EndMap;

            std::ofstream yaml_file(yaml_path);
            yaml_file << out.c_str();
        }
    }
}

/**
 *
 * SYSTEM ROS
 *
 * **/
void AutoLoadMapSystem::setupPublishers()
{
    point_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/map/current_map", 1);
    filter_timer_ =
        nh_.createTimer(ros::Duration(0.05), &AutoLoadMapSystem::filterMapData, this);

    init_pose_MGRS_ = nh_.advertise<geometry_msgs::PoseStamped>("/odom_MGRS", 10);
    tf_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/tf_map_enu", 10);
}

void AutoLoadMapSystem::setupSubscribers()
{
    gnss_pose_sub_ = nh_.subscribe("gnss_pose_cov", 1,
                                   &AutoLoadMapSystem::callbackGNSSPoseCov, this);
    point_ref_sub_ = nh_.subscribe("/point_ref", 10, &AutoLoadMapSystem::pointRefCallback, this);
    pose_subscriber_ = nh_.subscribe("/odom_MGRS", 10, &AutoLoadMapSystem::poseCallback, this);
}

void AutoLoadMapSystem::setupServices()
{
    points_update_client_ = nh_.serviceClient<vehicle_localization_util::updatePoints>("points_update_service");
    first_point_client = nh_.serviceClient<vehicle_localization_util::PoseWithCovarianceStamped>("monte_align_srv");
}

/**
 *
 * PRE LOADMAP
 * @brief callbackGNSSPoseCov
 * @brief poseCallback
 * @brief startMapUpdateThread
 * @brief pointRefCallback
 * **/

void AutoLoadMapSystem::callbackGNSSPoseCov(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_cov_msg_ptr)
{
    if (!initialpose_received_)
    {
        if (!pose_cov_msg_ptr)
        {
            ROS_ERROR("[AutoLoadMapSystem] GNSS pose covariance message pointer is null!");
            return;
        }
        if (!processing_gnss)
        {
            geometry_msgs::PoseStamped pose_init_mgrs_msg;
            pose_init_mgrs_msg.header = pose_cov_msg_ptr->header;
            pose_init_mgrs_msg.pose = pose_cov_msg_ptr->pose.pose;
            init_pose_MGRS_.publish(pose_init_mgrs_msg);
            ROS_INFO("Pose init in MGRS: x=%.6f,y=%.6f,z=%.6f",
                     pose_cov_msg_ptr->pose.pose.position.x,
                     pose_cov_msg_ptr->pose.pose.position.y,
                     pose_cov_msg_ptr->pose.pose.position.z);
        }
        gnss_sub_data_ = *pose_cov_msg_ptr;

        std::thread gnss_thread([this, pose_cov_msg_ptr]()
                                {
            std::lock_guard<std::mutex> lock(gnss_mutex);
            gnss_queue.push(*pose_cov_msg_ptr);
            while (gnss_queue.size() > 2)
            {
                gnss_queue.pop();
            }
            if (gnss_queue.size() == 2)
            {
                auto first_point = gnss_queue.front();
                auto second_point = gnss_queue.back();
                if (second_point.header.stamp < first_point.header.stamp)
                {
                    ROS_INFO("[callbackGNSSPoseCov] GNSS data is not time synchronized. "
                             "Removing the later point...");
                    std::queue<geometry_msgs::PoseWithCovarianceStamped> temp_queue;
                    temp_queue.push(first_point);
                    std::swap(gnss_queue, temp_queue);
                    return;
                }
                double delta_x = second_point.pose.pose.position.x - first_point.pose.pose.position.x;
                double delta_y = second_point.pose.pose.position.y - first_point.pose.pose.position.y;
                double distance = sqrt(delta_x * delta_x + delta_y * delta_y);

                if (pub_tf_done_)
                {
                    if (distance <= 3.0)
                    {
                        std::cout << "[callbackGNSSPoseCov] Distance GNSS after processing: "<< distance << std::endl;
                        geometry_msgs::PoseWithCovarianceStamped result = second_point;

                        if (!processGNSSData(result) && !initialpose_received_)
                        {
                            gnss_queue.pop();
                            return;
                        }
                        else
                        {
                            ROS_INFO("[callbackGNSSPoseCov] Converged");
                            initialpose_received_ = true;
                        }
                    }
                    if (distance > 3.0)
                    {
                        gnss_queue.pop();
                        std::cout << "[callbackGNSSPoseCov] The GNSS signal is noisy" << std::endl;
                    }
                }
            } });

        if (gnss_thread.joinable())
        {
            gnss_thread.join();
        }
        processing_gnss = true;

        auto end_time = std::chrono::high_resolution_clock::now();
    }
}

bool AutoLoadMapSystem::processGNSSData(geometry_msgs::PoseWithCovarianceStamped &pose_cov_msg)
{
    // Apply transformation to GNSS coordinates
    tf2::Vector3 point(pose_cov_msg.pose.pose.position.x,
                       pose_cov_msg.pose.pose.position.y,
                       pose_cov_msg.pose.pose.position.z);

    // Thread-safe access to current_merged_cloud_
    pcl::PointCloud<PointT>::Ptr cloud_copy;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (current_merged_cloud_ && !current_merged_cloud_->empty())
        {
            cloud_copy.reset(new pcl::PointCloud<PointT>(*current_merged_cloud_));
        }
    }

    point.setZ(getGroundHeight(cloud_copy, point));

    pose_cov_msg.pose.pose.position.x = point.getX();
    pose_cov_msg.pose.pose.position.y = point.getY();
    pose_cov_msg.pose.pose.position.z = point.getZ();

    // Send pose to NDT Align Service
    geometry_msgs::PoseWithCovarianceStamped::Ptr aligned_pose_msg_ptr(
        new geometry_msgs::PoseWithCovarianceStamped);
    const bool succeeded_align = AutoLoadMapSystem::callInitService(pose_cov_msg, aligned_pose_msg_ptr);
    if (succeeded_align)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool AutoLoadMapSystem::callInitService(
    const geometry_msgs::PoseWithCovarianceStamped &input_pose_msg,
    const geometry_msgs::PoseWithCovarianceStamped::Ptr &output_pose_msg_ptr)
{
    vehicle_localization_util::PoseWithCovarianceStamped srv;
    srv.request.pose_with_cov = input_pose_msg;
    ROS_INFO("[callInitService] call NDT Align Server");
    if (first_point_client.call(srv))
    {
        std::cout << "***************callInitService success***************" << std::endl;
        *output_pose_msg_ptr = srv.response.pose_with_cov;
        resuilt_init_ = *output_pose_msg_ptr;
        return true;
    }
    else
    {
        ROS_WARN("[pose_initializer]++++++++++++++++Process failed+++++++++++.");
        return false;
    }
}

void AutoLoadMapSystem::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    float current_x = msg->pose.position.x;
    float current_y = msg->pose.position.y;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        position_queue_.push({current_x, current_y});
    }
    map_update_cv_.notify_one();
}

void AutoLoadMapSystem::startMapUpdateThread()
{
    map_update_thread_ = std::thread([this]()
                                     {
        while(!should_stop_){
            std::pair<float,float> current_position;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                map_update_cv_.wait(lock,[this](){
                    return !position_queue_.empty() || should_stop_;
                });
                if (should_stop_) break; 
                current_position = position_queue_.front(); // FIFO
                position_queue_.pop();
            }
            processMapUpdate(current_position.first, current_position.second);
        } });
}

void AutoLoadMapSystem::pointRefCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!origin_set_ && !first_point_)
    {
        origin_lat_ = msg->latitude;
        origin_lon_ = msg->longitude;
        origin_height_ = msg->altitude;

        local_cartesian_.Reset(origin_lat_, origin_lon_, origin_height_);
        origin_set_ = true;
        first_point_ = true;
        ROS_INFO("[pointRefCallback] Received lat,long,altitude");
    }
}

/**
 *
 * EXTRACTING INFORMATION FROM A BOUNDARY YAML FILE
 * @brief detectCurrentArea
 * @brief findYamlConfig
 * @brief loadMapConfigurations
 * @brief loadAndUpdateArea
 * @brief isMapNeeded
 * @brief calculateDistanceToMap
 * @brief pointToLineDistance
 * @brief calculateDistance
 *
 * **/
AreaInfo AutoLoadMapSystem::detectCurrentArea(float x, float y)
{
    AreaInfo result;
    std::string boundaries_file = base_pcd_path_ + "/map_boundaries_area.yaml";

    if (!fs::exists(boundaries_file))
    {
        return result;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(boundaries_file);
        if (!config["areas"])
        {
            ROS_ERROR("[detectCurrentArea] No areas defined in boundaries file");
            return result;
        }

        for (const auto &area : config["areas"])
        {
            std::string area_name = area["name"].as<std::string>();
            auto min = area["min"].as<std::vector<float>>();
            auto max = area["max"].as<std::vector<float>>();

            if (x >= min[0] && x <= max[0] && y >= min[1] && y <= max[1])
            {
                result.area_name = area_name; // ha dong,thanh xuan
                result.full_path = base_pcd_path_ + "/" + area_name;
                current_area_ = result.full_path; // hadong or thanh xuan
                findYamlConfig(current_area_);
                return result;
            }
        }

        ROS_WARN("[detectCurrentArea] Position (%.4f, %.4f) not found in any defined area", x, y);
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("[detectCurrentArea] Error parsing YAML file: %s", e.what());
    }

    return result;
}

void AutoLoadMapSystem::findYamlConfig(const std::string &area_path)
{
    try
    {
        std::string boundaries_path = area_path + "/map_boundaries.yaml";
        // Load and validate the YAML file
        YAML::Node config = YAML::LoadFile(boundaries_path);
        if (!config["files"])
        {
            ROS_ERROR("[findYamlConfig] Invalid YAML structure in %s",
                      boundaries_path.c_str());
            return;
        }

        if (!loaded_boundaries_file_)
        {
            loadMapConfigurations(boundaries_path);
        }
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("[findYamlConfig] YAML error: %s", e.what());
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[findYamlConfig] Error: %s", e.what());
    }
}

void AutoLoadMapSystem::loadMapConfigurations(const std::string &map_config_path_)
{
    try
    {
        YAML::Node yaml_file = YAML::LoadFile(map_config_path_);
        for (const auto &file : yaml_file["files"])
        {
            AreaInfo map_info;
            map_info.area_name = file["file"].as<std::string>();
            auto min = file["min"];
            auto max = file["max"];

            map_info.boundaries.x_min = min[0].as<float>();
            map_info.boundaries.y_min = min[1].as<float>();
            map_info.boundaries.x_max = max[0].as<float>();
            map_info.boundaries.y_max = max[1].as<float>();
            map_info.isLoaded = false;
            map_info.cloud.reset(new pcl::PointCloud<PointT>);

            available_maps_.push_back(map_info);
        }
        loaded_boundaries_file_ = true;
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("[AutoLoadMapSystem] Error loading map configurations: %s", e.what());
    }
}

bool AutoLoadMapSystem::loadAndUpdateArea(const AreaInfo &area_info)
{
    if (area_info.area_name.empty())
    {
        ROS_INFO("[loadAndUpdateArea] Currently the car is not in %s area !!!! ",
                 area_info.area_name.c_str());
        return false;
    }
    if (current_area_ == area_info.full_path)
    {
        return false;
    }

    std::string yaml_path = area_info.full_path + "/map_boundaries_area.yaml";
    if (yaml_path.empty())
    {
        return false;
    }
    current_area_ = area_info.full_path;
    return true;
}

bool AutoLoadMapSystem::isVehicleInMap(float x, float y, const AreaInfo &map)
{
    return (x >= map.boundaries.x_min && x <= map.boundaries.x_max &&
            y >= map.boundaries.y_min && y <= map.boundaries.y_max);
}

bool AutoLoadMapSystem::isMapNeeded(float x, float y, const AreaInfo &map, float &distance)
{
    distance = calculateDistanceToMap(x, y, map);
    return distance <= 50.0f; // Load maps in [35, 50] meters range
}

float AutoLoadMapSystem::calculateDistanceToMap(float x, float y, const AreaInfo &map)
{
    // Calculate distances to map boundaries
    float dist_to_left =
        pointToLineDistance(x, y, map.boundaries.x_min, map.boundaries.y_min, map.boundaries.x_min, map.boundaries.y_max);
    float dist_to_right =
        pointToLineDistance(x, y, map.boundaries.x_max, map.boundaries.y_min, map.boundaries.x_max, map.boundaries.y_max);
    float dist_to_bottom =
        pointToLineDistance(x, y, map.boundaries.x_min, map.boundaries.y_min, map.boundaries.x_max, map.boundaries.y_min);
    float dist_to_top =
        pointToLineDistance(x, y, map.boundaries.x_min, map.boundaries.y_max, map.boundaries.x_max, map.boundaries.y_max);

    // Return minimum distance to any boundary
    return std::min({dist_to_left, dist_to_right, dist_to_bottom, dist_to_top});
}

float calculateDistance(float x1, float x2, float y1, float y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}
/**
 * Calculate the distance from a point P(px, py) to line segment AB(x1,y1 → x2,y2).
 * vector:
 *        AP = (px - x1, py - y1)
 *        AB = (x2 - x1, y2 - y1)
 *  - Calculate the projection ratio (param) of AP onto AB:
 *        param = (AP · AB) / |AB|²
 *  - There are 3 cases:
 *      param < 0 → the closest point is A
 *      param > 1 → the closest point is B
 *      0 ≤ param ≤ 1 → the closest point H lies on AB
 *
 *  --> Finally: distance = |PH| :is the shortest distance to line segment AB
 * 
 *           P(px,py)
 *             *
 *            /|
 *           / |
 *     A *-----------------* B
 *             H (projected point)
 */
float AutoLoadMapSystem::pointToLineDistance(float px, float py, float x1, float y1, float x2, float y2)
{
    float A = px - x1; // x-component of vector AP (v1)
    float B = py - y1; // y-component of vector AP (v1)
    float C = x2 - x1; // x-component of vector AB (v2)
    float D = y2 - y1; // y-component of vector AB (v2)

    float dot = A * C + B * D;    // profitlessg (xv1.xv2+yv1.yv2)
    float len_sq = C * C + D * D; // |AB|²

    // Find projection points
    float param = -1; // ratio of determining the position of point H on line segment AB
    if (len_sq != 0)
        param = dot / len_sq;

    float xx, yy;

    if (param < 0)
    {
        xx = x1;
        yy = y1;
    }
    else if (param > 1)
    {
        xx = x2;
        yy = y2;
    }
    else
    { // The closest point lies on the line segment.
        xx = x1 + param * C;
        yy = y1 + param * D;
    }

    return calculateDistance(px, xx, py, yy);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
AutoLoadMapSystem::filterWithVoxelGrid(pcl::PointCloud<PointT>::Ptr cloud)
{
    if (!cloud || cloud->empty())
    {
        ROS_ERROR("[MapAutoLoad] Input cloud is null or empty for voxel filtering");
        return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.25f, 0.25f, 0.25f);
    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
    voxel_filter.filter(*filtered_cloud);

    if (filtered_cloud->empty())
    {
        ROS_ERROR("[MapAutoLoad] VoxelGrid filter produced an empty cloud. Returning original...");
        return cloud;
    }
    return filtered_cloud;
}

/**
 *
 * UPDATE MAP
 * @brief processMapUpdate
 * @brief loadMapAsync
 * @brief unloadMapAsync
 * @brief getGroundHeight
 * @brief notifyPointsUpdate
 * @brief filterMapData
 *
 * **/
void AutoLoadMapSystem::processMapUpdate(float current_x, float current_y)
{
    auto start_total = std::chrono::high_resolution_clock::now();

    // detect area
    auto start = std::chrono::high_resolution_clock::now();
    AreaInfo current_area = detectCurrentArea(current_x, current_y);
    auto detect_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_detect = detect_time - start;

    if (loadAndUpdateArea(current_area))
    {
        ROS_INFO("[processMapUpdate] Switched to area: %s", current_area.area_name.c_str());
    }

    std::vector<std::string> maps_to_load;
    std::vector<std::string> maps_to_unload;
    std::vector<std::string> maps_to_publish;
    bool needs_update = false;

    {
        std::lock_guard<std::mutex> lock(map_state_mutex_);
        for (auto &map : available_maps_)
        {
            float distance;
            bool is_current_map = isVehicleInMap(current_x, current_y, map);
            bool should_be_loaded = isMapNeeded(current_x, current_y, map, distance);
            bool is_already_loaded = (loaded_clouds_.find(map.area_name) != loaded_clouds_.end());

            if (!map.isLoaded)
            {
                if (should_be_loaded && !is_already_loaded)
                {
                    maps_to_load.push_back(map.area_name);
                    map.isLoaded = true;
                    map.isPublish = false;
                    ROS_INFO("[processMapUpdate] Queuing map %s for loading to buffer (distance: %.4f m)",
                             map.area_name.c_str(), distance);
                }
            }

            if (map.isLoaded)
            {
                if (distance > 60.0f && !is_current_map)
                {
                    maps_to_unload.push_back(map.area_name);
                    maps_to_load.erase(std::remove(maps_to_load.begin(), maps_to_load.end(), map.area_name),
                                       maps_to_load.end());
                    maps_to_publish.erase(std::remove(maps_to_publish.begin(), maps_to_publish.end(), map.area_name),
                                          maps_to_publish.end());
                    map.isLoaded = false;
                    map.isPublish = false;
                    ROS_INFO("[processMapUpdate] Removing map %s  (distance: %.4f m)", map.area_name.c_str(), distance);
                }

                if (distance <= 33.0f && !map.isPublish)
                {
                    maps_to_publish.push_back(map.area_name);
                }
            }
        }
    }

    needs_update = true;
    if (needs_update)
    {
        std::vector<std::thread> threads;
        for (const auto &filename : maps_to_load)
        {
            threads.emplace_back([this, filename]()
                                 { loadMapAsync(filename); });
        }

        for (const auto &filename : maps_to_unload)
        {
            threads.emplace_back([this, filename]()
                                 { unloadMapAsync(filename); });
        }

        for (auto &thread : threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }

        start = std::chrono::high_resolution_clock::now();
        updateMergedCloud(maps_to_publish);
        auto update_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration_update = update_time - start;
        ROS_INFO("[processMapUpdate] Time to update %.4f seconds", duration_update.count());
    }

    auto end_time_total = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time_total - start_total;
}

void AutoLoadMapSystem::loadMapAsync(const std::string &filename)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    auto start_time = std::chrono::high_resolution_clock::now();

    if (pcl::io::loadPCDFile<PointT>(current_area_ + "/pcd/" + filename, *cloud) == -1)
    {
        ROS_ERROR("[AutoLoadMapSystem] Failed to load PCD file: %s", filename.c_str());
        return;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> load_duration = end_time - start_time;
    ROS_INFO("[loadMapAsync] Time to load map %s : %.4f seconds", filename.c_str(), load_duration.count());

    {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        if (loaded_clouds_.find(filename) != loaded_clouds_.end())
        {
            return;
        }

        auto filtered_cloud = cloud; // Skip filtering for now to avoid potential issues
        if (filtered_cloud && !filtered_cloud->empty())
        {
            loaded_clouds_[filename] = filtered_cloud;
        }
        else
        {
            ROS_ERROR("[loadMapAsync] Filtered cloud is empty for %s", filename.c_str());
        }
    }
}

void AutoLoadMapSystem::unloadMapAsync(const std::string &filename)
{
    {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        auto it = loaded_clouds_.find(filename);
        if (it == loaded_clouds_.end())
        {
            ROS_ERROR("[unloadMapAsync] Map %s not found in loaded_clouds_", filename.c_str());
            return;
        }

        if (it->second && !it->second->empty())
        {
            total_points_removed += it->second->size();
            if (removed_points_cloud)
            {
                *removed_points_cloud += *(it->second);
            }
        }
        loaded_clouds_.erase(it);
    }
}

double AutoLoadMapSystem::getGroundHeight(const pcl::PointCloud<PointT>::Ptr &pcdmap,
                                          const tf2::Vector3 &point)
{
    constexpr double radius = 1.0;
    const double x = point.getX();
    const double y = point.getY();

    double height = INFINITY;

    if (pcdmap && !pcdmap->empty())
    {
        int points_within_radius = 0;
        std::map<std::pair<int, int>, std::vector<double>> grid_height_map;

        // Grid-based Search
        for (const auto &p : pcdmap->points)
        {
            double dx = x - p.x;
            double dy = y - p.y;
            double distance_squared = (dx * dx) + (dy * dy);

            if (distance_squared < radius * radius)
            {
                points_within_radius++;
                int grid_x = static_cast<int>(p.x * 10);
                int grid_y = static_cast<int>(p.y * 10);
                grid_height_map[{grid_x, grid_y}].push_back(p.z);
            }
        }

        // Calculate median height in each grid cell and choose the lowest one
        for (const auto &entry : grid_height_map)
        {
            const auto &heights = entry.second;
            if (!heights.empty())
            {
                double median_height;
                std::vector<double> sorted_heights = heights;
                std::sort(sorted_heights.begin(), sorted_heights.end());
                if (sorted_heights.size() % 2 == 0)
                {
                    median_height = (sorted_heights[sorted_heights.size() / 2 - 1] +
                                     sorted_heights[sorted_heights.size() / 2]) /
                                    2;
                }
                else
                {
                    median_height = sorted_heights[sorted_heights.size() / 2];
                }
                height = std::min(height, median_height);
            }
        }
    }
    else
    {
        ROS_WARN("[Preprocessing] Filtered PointCloud data is not available or empty.");
    }

    if (!std::isfinite(height))
    {
        ROS_WARN("[Auto-Preprocessing] Could not determine ground height, using GNSS height.");
        height = point.getZ();
    }

    return height;
}

void AutoLoadMapSystem::updateMergedCloud(const std::vector<std::string> &maps_to_publish)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    std::unique_lock<std::mutex> merge_lock(merge_mutex_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr new_merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    size_t total_points_added = 0;

    std::unordered_map<std::string, pcl::PointCloud<PointT>::Ptr> cloud_copies;
    {
        std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
        for (const auto &pair : loaded_clouds_)
        {
            if (pair.second && !pair.second->empty())
            {
                cloud_copies[pair.first] = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(*pair.second));
            }
        }
    }

    // Merge all loaded maps
    std::unordered_set<std::string> active_maps;
    {
        std::lock_guard<std::mutex> state_lock(map_state_mutex_);
        for (const auto &map : available_maps_)
        {
            if (map.isLoaded && cloud_copies.find(map.area_name) != cloud_copies.end())
            {
                active_maps.insert(map.area_name);
            }
        }
    }

    for (const auto &filename : active_maps)
    {
        auto it = cloud_copies.find(filename);
        if (it != cloud_copies.end() && it->second && !it->second->empty())
        {
            *new_merged_cloud += *(it->second);
        }
    }

    // Process maps_to_publish
    {
        std::lock_guard<std::mutex> state_lock(map_state_mutex_);
        for (const auto &filename : maps_to_publish)
        {
            auto it = cloud_copies.find(filename);
            if (it != cloud_copies.end() && it->second && !it->second->empty())
            {
                bool is_new_publish = false;
                for (auto &map : available_maps_)
                {
                    if (map.area_name == filename && !map.isPublish && map.isLoaded)
                    {
                        map.isPublish = true;
                        is_new_publish = true;
                        break;
                    }
                }
                if (is_new_publish)
                {
                    total_points_added += it->second->size();
                    if (added_points_cloud)
                    {
                        *added_points_cloud += *(it->second);
                    }
                    ROS_INFO("[updateMergedCloud] Added %zu points from map %s for publishing",
                             it->second->size(), filename.c_str());
                }
            }
            else
            {
                ROS_WARN("[updateMergedCloud] Map %s in maps_to_publish but not in loaded_clouds_", filename.c_str());
            }
        }
    }

    if (!removed_points_cloud)
    {
        removed_points_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        ROS_INFO("[updateMergedCloud] Initialized removed_points_cloud");
    }

    // Thread-safe update of current_merged_cloud_
    {
        std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
        current_merged_cloud_ = new_merged_cloud;
    }

    // Publish merged cloud
    if (!new_merged_cloud->empty() || total_points_removed > 0)
    {
        sensor_msgs::PointCloud2 cloud_msg;
        try
        {
            pcl::toROSMsg(*new_merged_cloud, cloud_msg);
            cloud_msg.header.frame_id = "map";
            cloud_msg.header.stamp = ros::Time::now();
            point_cloud_pub_.publish(cloud_msg);
            ROS_INFO("[updateMergedCloud] Published merged cloud with %zu points, removed %zu points",
                     new_merged_cloud->size(), total_points_removed);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("[updateMergedCloud] Error publishing cloud: %s", e.what());
            return;
        }

        auto tf_time = std::chrono::high_resolution_clock::now();
        if (!pub_tf_done_)
        {
            std::lock_guard<std::mutex> tf_lock(mtx_tf);
            tf2::Vector3 point(gnss_sub_data_.pose.pose.position.x,
                               gnss_sub_data_.pose.pose.position.y,
                               gnss_sub_data_.pose.pose.position.z);

            // Thread-safe access to current_merged_cloud_
            pcl::PointCloud<PointT>::Ptr cloud_copy;
            {
                std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
                if (current_merged_cloud_ && !current_merged_cloud_->empty())
                {
                    cloud_copy.reset(new pcl::PointCloud<PointT>(*current_merged_cloud_));
                }
            }

            point.setZ(getGroundHeight(cloud_copy, point));
            gnss_sub_data_.pose.pose.position.x = point.getX();
            gnss_sub_data_.pose.pose.position.y = point.getY();
            gnss_sub_data_.pose.pose.position.z = point.getZ();

            ROS_INFO("[updateMergedCloud] Origin set in MGRS: %.6f, %.6f, %.6f",
                     gnss_sub_data_.pose.pose.position.x,
                     gnss_sub_data_.pose.pose.position.y,
                     gnss_sub_data_.pose.pose.position.z);

            double enu_x, enu_y, enu_z;
            local_cartesian_.Forward(origin_lat_, origin_lon_, origin_height_, enu_x, enu_y, enu_z);

            transform_map_enu_.header.stamp = ros::Time::now();
            transform_map_enu_.header.frame_id = "map";
            transform_map_enu_.child_frame_id = "enu";
            transform_map_enu_.transform.translation.x = -gnss_sub_data_.pose.pose.position.x;
            transform_map_enu_.transform.translation.y = -gnss_sub_data_.pose.pose.position.y;
            transform_map_enu_.transform.translation.z = -gnss_sub_data_.pose.pose.position.z;
            transform_map_enu_.transform.rotation.x = 0.0;
            transform_map_enu_.transform.rotation.y = 0.0;
            transform_map_enu_.transform.rotation.z = 0.0;
            transform_map_enu_.transform.rotation.w = 1.0;

            tf_pub_.publish(transform_map_enu_);
            ROS_INFO("[updateMergedCloud] Published TF transform");
            pub_tf_done_ = true;
        }
        auto end_tf_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_tf = end_tf_time - tf_time;

        notifyPointsUpdate(total_points_added, total_points_removed, added_points_cloud, removed_points_cloud);
        total_points_removed = 0;
    }
    else
    {
        sensor_msgs::PointCloud2 empty_msg;
        empty_msg.header.frame_id = "map";
        empty_msg.header.stamp = ros::Time::now();
        point_cloud_pub_.publish(empty_msg);
        ROS_WARN("[updateMergedCloud] Published empty cloud (no valid maps to publish, no points removed)");
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    ROS_INFO("[updateMergedCloud] Cloud update completed in %.4f seconds", duration.count());
}

bool AutoLoadMapSystem::notifyPointsUpdate(size_t points_added, size_t points_removed,
                                           const pcl::PointCloud<pcl::PointXYZI>::Ptr &added_cloud,
                                           const pcl::PointCloud<pcl::PointXYZI>::Ptr &removed_cloud)
{
    ROS_DEBUG("[MapAutoLoad] notifyPointsUpdate: %zu added, %zu removed", points_added, points_removed);
    ROS_DEBUG("Added cloud: %p, size: %zu", added_cloud.get(), added_cloud ? added_cloud->size() : 0);
    ROS_DEBUG("Removed cloud: %p, size: %zu", removed_cloud.get(), removed_cloud ? removed_cloud->size() : 0);

    auto start_time = std::chrono::high_resolution_clock::now();

    if (points_added == 0 && points_removed == 0)
    {
        ROS_DEBUG("[MapAutoLoad] No points added or removed, skipping service call");
        return true;
    }

    if (!points_update_client_)
    {
        ROS_WARN("[MapAutoLoad] Points update service client not initialized");
        return false;
    }

    if (!points_update_client_.waitForExistence(ros::Duration(2.0)))
    {
        ROS_WARN("[MapAutoLoad] Points update service not available");
        return false;
    }

    vehicle_localization_util::updatePoints srv;
    srv.request.points_added = points_added;
    srv.request.points_removed = points_removed;

    try
    {
        if (points_added > 0 && added_cloud && !added_cloud->empty())
        {
            ROS_DEBUG("Converting added_cloud to ROS message");
            pcl::toROSMsg(*added_cloud, srv.request.cloud_data.added_points);
            srv.request.cloud_data.added_points.header.frame_id = "map";
            srv.request.cloud_data.added_points.header.stamp = ros::Time::now();
        }

        if (points_removed > 0 && removed_cloud && !removed_cloud->empty())
        {
            ROS_DEBUG("Converting removed_cloud to ROS message");
            pcl::toROSMsg(*removed_cloud, srv.request.cloud_data.removed_points);
            srv.request.cloud_data.removed_points.header.frame_id = "map";
            srv.request.cloud_data.removed_points.header.stamp = ros::Time::now();
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception during point cloud conversion: %s", e.what());
        return false;
    }

    bool success = false;
    if (points_update_client_.call(srv))
    {
        ROS_INFO("[MapAutoLoad] Successfully notified points update: %zu added, %zu removed",
                 points_added, points_removed);
        success = true;
    }
    else
    {
        ROS_ERROR("[MapAutoLoad] Failed to call points update service");
        success = false;
    }

    // Clean up point clouds regardless of service call result
    if (added_points_cloud)
    {
        added_points_cloud->clear();
    }
    if (removed_points_cloud)
    {
        removed_points_cloud->clear();
    }
    total_points_added = 0;
    total_points_removed = 0;

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> load_duration = end_time - start_time;
    ROS_INFO("[notifyPointsUpdate] Time to process serviceUpdate: %.4f seconds", load_duration.count());

    return success;
}

void AutoLoadMapSystem::filterMapData(const ros::TimerEvent &)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (!current_merged_cloud_ || current_merged_cloud_->empty())
        {
            return;
        }

        try
        {
            cloud_copy.reset(new pcl::PointCloud<pcl::PointXYZI>(*current_merged_cloud_));
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("[filterMapData] Error creating cloud copy: %s", e.what());
            return;
        }
    }

    if (!cloud_copy || cloud_copy->empty())
    {
        return;
    }

    has_map = true;
    
    try
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_copy, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = ros::Time::now();
        point_cloud_pub_.publish(cloud_msg);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[filterMapData] Error publishing cloud: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_auto_load");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AutoLoadMapSystem map_auto_load(nh, private_nh);
    ROS_INFO("[AutoLoadMapSystem] Ready to load map automatically.");

    ros::spin();
    return 0;
}