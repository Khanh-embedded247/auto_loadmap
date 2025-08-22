#include "auto_loadmap/auto_load_v1.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <limits>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <thread>

namespace fs = boost::filesystem;

MapAutoLoad::MapAutoLoad(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh), added_points_cloud(new pcl::PointCloud<pcl::PointXYZI>),
      removed_points_cloud(new pcl::PointCloud<pcl::PointXYZI>), should_stop_(false),
      start_time_gnss(std::chrono::high_resolution_clock::now()), tf_buffer_(), tf_listener_(tf_buffer_)
{
  private_nh_.param<std::string>("map_config_path", map_config_path_,
                                 "/workspaces/fusion_ekf/Localization_Indoor/src/hdl_localization/map/config");
  private_nh_.param<std::string>("base_pcd_path", base_pcd_path_,
                                 "/workspaces/fusion_ekf/Localization_Indoor/src/hdl_localization/map/config");
  setupPublishers();
  setupSubscribers();
  setupServices();

  ROS_INFO("[MapAutoLoad] Node initialized.");
  for (const auto &subdir : fs::directory_iterator(base_pcd_path_))
  {
    if (fs::is_directory(subdir))
    {
      std::string subdir_path = subdir.path().string();
      std::string yaml_path = subdir_path + "/map_boundaries.yaml";
      std::cout << "Checking path: " << yaml_path << std::endl;
      if (!fs::exists(yaml_path))
      {
        generateYamlForArea(base_pcd_path_);
      }
      std::cout << "Checked: " << yaml_path << std::endl;
    }
  }

  startMapUpdateThread();
}

MapAutoLoad::~MapAutoLoad() {}

void MapAutoLoad::setupPublishers()
{
  point_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/map/current_map", 1);
  filter_timer_ =
      nh_.createTimer(ros::Duration(0.05), &MapAutoLoad::filterMapData, this);

  point_filter_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/map/auto_filter", 1);
  init_pose_MGRS_ = nh_.advertise<geometry_msgs::PoseStamped>("/odom_MGRS", 10);
  tf_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/tf_map_enu", 10);
}

void MapAutoLoad::setupSubscribers()
{
  gnss_pose_sub_ = nh_.subscribe("gnss_pose_cov", 1,
                                 &MapAutoLoad::callbackGNSSPoseCov, this);
  point_ref_sub_ = nh_.subscribe("/point_ref", 10, &MapAutoLoad::pointRefCallback, this);
  pose_subscriber_ =
      nh_.subscribe("/odom_MGRS", 10, &MapAutoLoad::poseCallback, this);
}
void MapAutoLoad::setupServices()
{
  points_update_client_ = nh_.serviceClient<vehicle_localization_util::updatePoints>("points_update_service");
}
void MapAutoLoad::startMapUpdateThread() // xử lý khi xe đứng im mà callback cứ
                                         // chạy
{
  map_update_thread_ = std::thread([this]()
                                   {
    while (!should_stop_) {
      auto start_time = std::chrono::high_resolution_clock::now();
      std::pair<float, float> current_position; // x,y
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        map_update_cv_.wait(
            lock,
            [this]() { // Pause the thread and wait for a condition to occur.
              return !position_queue_.empty() || should_stop_;
            });

        current_position = position_queue_.front(); // FIFO
        position_queue_.pop();
      }
      processMapUpdate(current_position.first, current_position.second);
      auto end_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> load_duration = end_time - start_time;
      // ROS_INFO("[startMapUpdateThread] Time to process map : %.2f seconds",
      // load_duration.count());
    } });
}
void MapAutoLoad::generateYamlForArea(const std::string &input_pcd)
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
          pcl::PointCloud<PointT>::Ptr cloud(
              new pcl::PointCloud<PointT>);

          if (pcl::io::loadPCDFile<PointT>(entry.path().string(),
                                           *cloud) == -1)
          {
            ROS_ERROR("[MapAutoLoad] Failed to load PCD file: %s",
                      entry.path().string().c_str());
            return;
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
          ROS_INFO(
              "[generateYamlForArea] Created successfully %s boundaries map",
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
      // ROS_INFO("[generateYamlForArea] Created map boundaries for %s YAML file
      // at %s", subdir_path.c_str(), yaml_path.c_str());
    }
  }
}

AreaInfo MapAutoLoad::detectCurrentArea(float x, float y)
{
  AreaInfo result;
  std::string boundaries_file = base_pcd_path_ + "/map_boundaries_area.yaml";

  if (!fs::exists(boundaries_file))
  {
    // ROS_ERROR("[detectCurrentArea] Main boundaries file not found at: %s",
    // boundaries_file.c_str());
    return result;
  }
  // else
  // {

  //     ROS_INFO("[detectCurrentArea] AREA boundaries file found at: %s",
  //     boundaries_file.c_str());
  // }

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
        current_area_ = result.full_path;

        // ROS_INFO("[detectCurrentArea] Current position (%.2f, %.2f) is in
        // area: %s",
        //          x, y, result.area_name.c_str());

        findYamlConfig(current_area_);
        return result;
      }
    }

    ROS_WARN(
        "[detectCurrentArea] Position (%.2f, %.2f) not found in any defined "
        "area",
        x, y);
  }
  catch (const YAML::Exception &e)
  {
    ROS_ERROR("[detectCurrentArea] Error parsing YAML file: %s", e.what());
  }

  return result;
}

void MapAutoLoad::findYamlConfig(const std::string &area_path)
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
    // available_maps_.clear();
    if (!loaded_boundaries_file_)
    {
      loadMapConfigurations(boundaries_path);
      // ROS_INFO("[findYamlConfig] Successfully processed boundaries for: %s",
      // area_path.c_str());
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

bool MapAutoLoad::loadAndUpdateArea(const AreaInfo &area_info)
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

pcl::PointCloud<pcl::PointXYZI>::Ptr
MapAutoLoad::filterWithVoxelGrid(pcl::PointCloud<PointT>::Ptr cloud)
{
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.3f, 0.3f, 0.3f);
  pcl::PointCloud<PointT>::Ptr filtered_cloud(
      new pcl::PointCloud<PointT>);
  voxel_filter.filter(*filtered_cloud);
  if (filtered_cloud->empty())
  {
    ROS_ERROR(
        "[MapAutoLoad] VoxelGrid filter produced an empty cloud. Skipping...");
    return cloud;
  }

  // Publish updated map
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*filtered_cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = ros::Time::now();
  //   point_filter_pub_.publish(cloud_msg);
  return filtered_cloud;
}
void MapAutoLoad::loadMapConfigurations(const std::string &map_config_path_)
{
  try
  {
    YAML::Node yaml_file = YAML::LoadFile(map_config_path_);
    for (const auto &file : yaml_file["files"])
    {
      MapInfo map_info;
      map_info.filename = file["file"].as<std::string>();
      auto min = file["min"];
      auto max = file["max"];

      map_info.xmin = min[0].as<float>();
      map_info.ymin = min[1].as<float>();
      map_info.xmax = max[0].as<float>();
      map_info.ymax = max[1].as<float>();
      map_info.is_loaded = false;
      map_info.cloud.reset(new pcl::PointCloud<PointT>);

      available_maps_.push_back(map_info);
    }
    loaded_boundaries_file_ = true;
    // ROS_INFO("[MapAutoLoad] Loaded %zu map configurations",
    // available_maps_.size());
  }
  catch (const YAML::Exception &e)
  {
    ROS_ERROR("[MapAutoLoad] Error loading map configurations: %s", e.what());
  }
}
float calculateDistance(float x1, float x2, float y1, float y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

std::string MapAutoLoad::getOSMFile(const std::string &pcd_file)
{
  std::string osm_file = pcd_file;
  size_t pos = osm_file.find(".pcd");
  if (pos != std::string::npos)
  {
    osm_file.replace(pos, 4, ".osm");
  }
  return osm_file;
}

float MapAutoLoad::pointToLineDistance(float px, float py, float x1, float y1,
                                       float x2, float y2)
{
  float A = px - x1; // thành phần x của vector AP (v₁)
  float B = py - y1; // thành phần y của vector AP (v₁)
  float C = x2 - x1; // thành phần x của vector AB (v₂)
  float D = y2 - y1; // thành phần y của vector AB (v₂)

  float dot = A * C + B * D;    // tich vo huong (xv1.xv2+yv1.yv2)
  float len_sq = C * C + D * D; // |AB|²

  // Tìm điểm chiếu
  float param = -1; // tỉ lệ xác định vị trí của điểm H trên đoạn thẳng AB
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
  { // Điểm gần nhất nằm trên đoạn thẳng
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  return calculateDistance(px, xx, py, yy);
}
float MapAutoLoad::calculateDistanceToMap(float x, float y,
                                          const MapInfo &map)
{
  // Calculate distances to map boundaries
  float dist_to_left =
      pointToLineDistance(x, y, map.xmin, map.ymin, map.xmin, map.ymax);
  float dist_to_right =
      pointToLineDistance(x, y, map.xmax, map.ymin, map.xmax, map.ymax);
  float dist_to_bottom =
      pointToLineDistance(x, y, map.xmin, map.ymin, map.xmax, map.ymin);
  float dist_to_top =
      pointToLineDistance(x, y, map.xmin, map.ymax, map.xmax, map.ymax);

  // Return minimum distance to any boundary
  return std::min({dist_to_left, dist_to_right, dist_to_bottom, dist_to_top});
}

bool MapAutoLoad::isMapNeeded(float x, float y, const MapInfo &map)
{
  if (x >= map.xmin && x <= map.xmax && y >= map.ymin && y <= map.ymax)
  {
    // ROS_INFO("Map %s is current ", map.filename.c_str());
    return true;
  }
  float dist = calculateDistanceToMap(x, y, map);
  return dist <= 70.0f;
}
void MapAutoLoad::processMapUpdate(float current_x, float current_y)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  // std::cout << "start detect" << std::endl;
  AreaInfo current_area = detectCurrentArea(current_x, current_y);

  if (loadAndUpdateArea(current_area))
  {
    ROS_INFO("Switched to area: %s", current_area.area_name.c_str());
  }

  std::vector<std::string> maps_to_load;
  std::vector<std::string> maps_to_unload;
  bool needs_update = false;

  {
    std::lock_guard<std::mutex> lock(map_state_mutex_);
    for (auto &map : available_maps_)
    {
      bool should_be_loaded = isMapNeeded(current_x, current_y, map);

      if (should_be_loaded && !map.is_loaded)
      {
        maps_to_load.push_back(map.filename);
        needs_update = true;
      }
      else if (!should_be_loaded && map.is_loaded)
      {
        maps_to_unload.push_back(map.filename);
        needs_update = true;
      }
    }
  }

  if (needs_update)
  {

    // std::vector<std::future<void>> tasks; // create a List that stores all
    // launched asynchronous jobs.
    std::vector<std::thread> threads;
    for (const auto &filename : maps_to_load)
    {
      // tasks.emplace_back(std::async(std::launch::async, [this, filename]()
      //                               { loadMapAsync(filename); }));
      threads.emplace_back([this, filename]()
                           {
        std::cout << "Thread ID (loading): " << std::this_thread::get_id()
                  << std::endl;
        loadMapAsync(filename); });
    }

    for (const auto &filename : maps_to_unload)
    {
      threads.emplace_back([this, filename]()
                           {
        std::cout << "Thread ID (unloading): " << std::this_thread::get_id()
                  << std::endl;
        unloadMapAsync(filename); });
    }

    for (auto &thread : threads)
    {
      if (thread.joinable())
      {
        thread.join();
      }
    }
    updateMergedCloud(maps_to_load, maps_to_unload);
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> load_duration = end_time - start_time;
  // ROS_INFO("[processMapUpdate] Time to process map : %.2f seconds",
  // load_duration.count());
}

void MapAutoLoad::loadMapAsync(const std::string &filename)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  auto start_time_load = std::chrono::high_resolution_clock::now();
  if (pcl::io::loadPCDFile<PointT>(current_area_ + "/pcd/" + filename,
                                   *cloud) == -1)
  {
    ROS_ERROR("[MapAutoLoad] Failed to load PCD file: %s", filename.c_str());
    return;
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> load_duration = end_time - start_time_load;
  ROS_INFO("[loadMapAsync] Time to load map %s : %.2f seconds",
           filename.c_str(), load_duration.count());

  {
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    auto filtered_cloud = filterWithVoxelGrid(cloud);
    // auto filtered_cloud = cloud;
    loaded_clouds_[filename] = filtered_cloud;
    lock.unlock();

    std::lock_guard<std::mutex> state_lock(map_state_mutex_);
    for (auto &map : available_maps_)
    {
      if (map.filename == filename)
      {
        map.is_loaded = true;
        ROS_DEBUG("[loadMapAsync] Marked %s as loaded", filename.c_str());
        break;
      }
    }
  }
}

void MapAutoLoad::unloadMapAsync(const std::string &filename)
{
  {
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    auto it = loaded_clouds_.find(filename);
    if (it == loaded_clouds_.end())
    {
      ROS_ERROR("[unloadMapAsync] Map %s not found in loaded_clouds_", filename.c_str());
      return;
    }
    if (!removed_points_cloud)
    {
      ROS_ERROR("[unloadMapAsync] removed_points_cloud is null, initializing");
      removed_points_cloud.reset(new pcl::PointCloud<PointT>);
    }
    total_points_removed += it->second->size();
    *removed_points_cloud += *(it->second);
    loaded_clouds_.erase(filename);

    // ROS_INFO("[MapAutoLoad] Map %s unloaded.", filename.c_str());
    std::lock_guard<std::mutex> state_lock(map_state_mutex_);
    for (auto &map : available_maps_)
    {
      if (map.filename == filename)
      {
        map.is_loaded = false;
        break;
      }
    }
  }
}
void MapAutoLoad::updateMergedCloud(
    const std::vector<std::string> &maps_to_load,
    const std::vector<std::string> &maps_to_unload)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  std::lock_guard<std::mutex> merge_lock(merge_mutex_);
  std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);

  pcl::PointCloud<PointT>::Ptr new_merged_cloud(
      new pcl::PointCloud<PointT>);

  //  find all maps that should be in the current view
  std::unordered_set<std::string> active_maps;
  for (const auto &map : available_maps_)
  {
    if (map.is_loaded && std::find(maps_to_unload.begin(), maps_to_unload.end(),
                                   map.filename) == maps_to_unload.end())
    {
      active_maps.insert(map.filename);
    }
  }

  // Add points from all active maps
  for (const auto &filename : active_maps)
  {
    auto it = loaded_clouds_.find(filename);
    if (it != loaded_clouds_.end())
    {
      *new_merged_cloud += *(it->second);
      ROS_INFO("[MapAutoLoad] Keeping map %s with %zu points", filename.c_str(),
               it->second->size());
    }
  }

  // Add points from newly loaded maps
  for (const auto &filename : maps_to_load)
  {
    auto it = loaded_clouds_.find(filename);
    if (it != loaded_clouds_.end())
    {
      *new_merged_cloud += *(it->second);
      *added_points_cloud += *(it->second);
      total_points_added += it->second->size();
      ROS_INFO("[MapAutoLoad] Added new map %s with %zu points",
               filename.c_str(), it->second->size());
    }
  }

  // Apply voxel grid filter to the complete new cloud
  if (!new_merged_cloud->empty())
  {
    current_merged_cloud_ = new_merged_cloud;
    if (!pub_tf_done_)
    {
      std::lock_guard<std::mutex> tf_mgrs_enu_(mtx_tf);
      tf2::Vector3 point(gnss_sub_data_.pose.pose.position.x,
                         gnss_sub_data_.pose.pose.position.y,
                         gnss_sub_data_.pose.pose.position.z);
      point.setZ(getGroundHeight(current_merged_cloud_, point));
      gnss_sub_data_.pose.pose.position.x = point.getX();
      gnss_sub_data_.pose.pose.position.y = point.getY();
      gnss_sub_data_.pose.pose.position.z = point.getZ();
      ROS_INFO("Origin set in MGRS: %.6f, %.6f, %.6f", gnss_sub_data_.pose.pose.position.x, gnss_sub_data_.pose.pose.position.y, gnss_sub_data_.pose.pose.position.z);
      // convert WGS84 to ENU
      double enu_x, enu_y, enu_z;
      local_cartesian_.Forward(origin_lat_, origin_lon_, origin_height_, enu_x, enu_y, enu_z);
      // ROS_INFO("Origin set at Lat: %.6f, Lon: %.6f, Height: %.2f,  Local: (%.2f, %.2f,%.2f)",
      //          origin_lat_, origin_lon_, origin_height_, enu_x, enu_y, enu_z);

      transform_map_enu_.header.stamp = ros::Time::now();
      transform_map_enu_.header.frame_id = "map"; // MGRS
      transform_map_enu_.child_frame_id = "enu";  // ENU
      transform_map_enu_.transform.translation.x = -gnss_sub_data_.pose.pose.position.x - 0;
      transform_map_enu_.transform.translation.y = -gnss_sub_data_.pose.pose.position.y - 0;
      transform_map_enu_.transform.translation.z = -gnss_sub_data_.pose.pose.position.z;
      transform_map_enu_.transform.rotation.x = 0.0;
      transform_map_enu_.transform.rotation.y = 0.0;
      transform_map_enu_.transform.rotation.z = 0.0;
      transform_map_enu_.transform.rotation.w = 1.0;

      tf_pub_.publish(transform_map_enu_);

      ROS_INFO("[MapAutoLoad] Pub tf.");
      // int num_points_to_print = std::min(10, static_cast<int>(current_merged_cloud_->size()));
      // ROS_INFO("Printing first %d points from raw cloud:", num_points_to_print);
      // for (int i = 0; i < num_points_to_print; i++)
      // {
      //   const PointT &point = current_merged_cloud_->points[i];
      //   ROS_INFO("RAW  Point %d: x=%.2f, y=%.2f, z=%.2f", i, point.x, point.y, point.z);
      // }
      pub_tf_done_ = true;
    }
  }
  else
  {
    sensor_msgs::PointCloud2 empty_msg;
    empty_msg.header.frame_id = "map";
    empty_msg.header.stamp = ros::Time::now();
    ROS_WARN("[MapAutoLoad] Published empty cloud after update");
  }
  notifyPointsUpdate(total_points_added, total_points_removed,
                     added_points_cloud, removed_points_cloud);
  // Log unload operations
  if (!maps_to_unload.empty())
  {
    ROS_INFO("[MapAutoLoad] Successfully unloaded maps:");
    for (const auto &filename : maps_to_unload)
    {
      ROS_INFO("  - %s", filename.c_str());
    }
  }
}
bool MapAutoLoad::notifyPointsUpdate(size_t points_added, size_t points_removed,
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

  if (points_update_client_.call(srv))
  {

    ROS_INFO("[MapAutoLoad] Successfully notified points update: %zu added, %zu removed", points_added, points_removed);
    added_points_cloud.reset(new pcl::PointCloud<PointT>);
    removed_points_cloud.reset(new pcl::PointCloud<PointT>);
    total_points_added = 0;
    total_points_removed = 0;
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> load_duration = end_time - start_time;
    ROS_INFO("[serviceUpdate] Time to process serviceUpdate: %.2f seconds", load_duration.count());
    return true;
  }
  else
  {
    ROS_ERROR("[MapAutoLoad] Failed to call points update service");
    added_points_cloud.reset(new pcl::PointCloud<PointT>);
    removed_points_cloud.reset(new pcl::PointCloud<PointT>);
    total_points_added = 0;
    total_points_removed = 0;
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> load_duration = end_time - start_time;
    ROS_INFO("[serviceUpdate] Time to process serviceUpdate: %.2f seconds", load_duration.count());

    return false;
  }
}
double MapAutoLoad::getGroundHeight(
    const pcl::PointCloud<PointT>::Ptr &pcdmap,
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
    ROS_INFO("[AutoLoadMap] Points found within radius: %d", points_within_radius);

    // Tính trung bình chiều cao trong mỗi ô lưới và chọn ô có chiều cao thấp
    // nhất
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

    std::cout << "[Preprocessing] GNSS Height after processing: " << height
              << std::endl;
  }
  else
  {
    ROS_WARN("[Preprocessing] Filtered PointCloud data is not available or empty.");
  }

  if (!std::isfinite(height))
  {
    ROS_WARN("[Auto-Preprocessing]Could not determine ground height, using GNSS "
             "height.");
    height = point.getZ();
  }

  return height;
}
void MapAutoLoad::filterMapData(const ros::TimerEvent &)
{

  if (!current_merged_cloud_)
  {
    // ROS_WARN("[pose_initializer] Map not loaded yet. Waiting...");
    return;
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*current_merged_cloud_, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = ros::Time::now();
  point_cloud_pub_.publish(cloud_msg);
  // ROS_INFO("size of point after loadmap: %d", current_merged_cloud_->width); // 45455472
}

void MapAutoLoad::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  float current_x = msg->pose.position.x;
  float current_y = msg->pose.position.y;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    position_queue_.push({current_x, current_y});
  }
  map_update_cv_.notify_one();
}
void MapAutoLoad::callbackGNSSPoseCov(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr
        &pose_cov_msg_ptr) // map
{

  if (!pose_cov_msg_ptr)
  {
    ROS_ERROR("[MapAutoLoad] GNSS pose covariance message pointer is null!");
    return;
  }
  gnss_sub_data_ = *pose_cov_msg_ptr;
  if (gnss_received_)
  {
    // ROS_WARN("[MapAutoLoad] GNSS data already received. Ignoring new
    // input.");
    return;
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> load_duration = end_time - start_time_gnss;
  ROS_INFO("[AUTOLOADMAP] Time to receivegnss : %.2f seconds",
           load_duration.count());
  geometry_msgs::PoseStamped pose_init_mgrs_msg;
  pose_init_mgrs_msg.header = pose_cov_msg_ptr->header;
  pose_init_mgrs_msg.pose = pose_cov_msg_ptr->pose.pose;
  init_pose_MGRS_.publish(pose_init_mgrs_msg);
  ROS_INFO("Pose init in MGRS: x=%.6f,y=%.6f,z=%.6f", pose_cov_msg_ptr->pose.pose.position.x, pose_cov_msg_ptr->pose.pose.position.y, pose_cov_msg_ptr->pose.pose.position.z);
  gnss_received_ = true;
}
void MapAutoLoad::pointRefCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  if (!origin_set_ && !first_point_)
  {
    origin_lat_ = msg->latitude;
    origin_lon_ = msg->longitude;
    origin_height_ = msg->altitude;

    local_cartesian_.Reset(origin_lat_, origin_lon_, origin_height_);
    origin_set_ = true;
    first_point_ = true;
    ROS_INFO("[pointRefCallback] Received lat,long,atitude");
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_auto_load");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  MapAutoLoad map_auto_load(nh, private_nh);
  ROS_INFO("[MapAutoLoad] Ready to load map automatically.");

  ros::spin();
  return 0;
}