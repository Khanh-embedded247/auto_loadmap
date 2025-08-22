#include "auto_loadmap/pre_processing.h"

Preprocessing::Preprocessing(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh),
      object_pre_(new pclomp::NormalDistributionsTransform<PointT, PointT>()),
      transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>),
      storage_points_around(new pcl::PointCloud<pcl::PointXYZI>),
      storage_points_enu(new pcl::PointCloud<pcl::PointXYZI>), map_frame("enu"),
      storage_points_MGRS(new pcl::PointCloud<pcl::PointXYZI>), tfListener(tfBuffer)
{
    object_pre_->setResolution(2.0);
    object_pre_->setStepSize(0.1);
    object_pre_->setTransformationEpsilon(0.01);
    object_pre_->setMaximumIterations(30);
    object_pre_->setNumThreads(4);
    setupPublishers();
    setupSubscribers();
    setupServices();
    private_nh_.param<float>("radius_max", radius_max_, 30.0);
    private_nh_.param<float>("filter_point_lidar", filter_point_, 3.0);
    private_nh_.param<double>("downsample_resolution", downsample_resolution, 0.3);
    private_nh_.param<int>("particles_num", temp_particles_num, 500);
    private_nh_.param<int>("n_startup_trials", temp_n_startup_trials, 50);
    params_.initial_pose_estimation.particles_num = temp_particles_num;
    params_.initial_pose_estimation.n_startup_trials = temp_n_startup_trials;
    params_.frame.map_frame = map_frame;
}
Preprocessing::~Preprocessing() {};
void Preprocessing::setupPublishers()
{
    point_enu_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_enu", 10);
    point_enu_around = nh_.advertise<sensor_msgs::PointCloud2>("/map_enu_around", 10);
    points_monte_aligned_pub = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_points_monte", 5, false);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("monte_markers", 1);
    init_monte_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
}
void Preprocessing::setupSubscribers()
{
    odom_enu_sub_ = nh_.subscribe("/odom_align", 1, &Preprocessing::callbackCreatePointAround, this);
    points_sub = nh_.subscribe("velodyne_points", 5, &Preprocessing::points_callback, this);
}
void Preprocessing::setupServices()
{
    points_update_server_ = nh_.advertiseService("points_update_service", &Preprocessing::serviceUpdate, this);
    send_local_map_client = nh_.serviceClient<vehicle_localization_util::SetLocalMap>("map_enu_update");
    first_point_server = nh_.advertiseService("monte_align_srv", &Preprocessing::serviceInit, this);
}
bool Preprocessing::serviceUpdate(
    vehicle_localization_util::updatePoints::Request &req,
    vehicle_localization_util::updatePoints::Response &res)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    ROS_INFO("[serviceUpdate] Storage start contains %zu points", storage_points_enu->size());
    ROS_INFO("[serviceUpdate] Received update: %lu points added, %lu points removed",
             (unsigned long)req.points_added, (unsigned long)req.points_removed);

    pcl::PointCloud<PointT>::Ptr removed_cloud_mgrs(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr removed_cloud_enu(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr added_cloud_mgrs(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr added_cloud_enu(new pcl::PointCloud<PointT>());

    try
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (req.points_removed > 0 && !req.cloud_data.removed_points.data.empty())
        {
            pcl::fromROSMsg(req.cloud_data.removed_points, *removed_cloud_mgrs);
            if (removed_cloud_mgrs->empty())
            {
                ROS_WARN("[serviceUpdate] Removed cloud is empty despite points_removed = %lu", req.points_removed);
            }
            else
            {
                if (!removed_cloud_mgrs->empty())
                {
                    // ROS_INFO("[serviceUpdate] Sample MGRS removed point: x=%.2f, y=%.2f, z=%.2f",
                    //          removed_cloud_mgrs->points[0].x, removed_cloud_mgrs->points[0].y, removed_cloud_mgrs->points[0].z);

                    *removed_cloud_enu = *tf_pointcloud_enu(removed_cloud_mgrs, removed_cloud_enu);
                    if (!removed_cloud_enu->empty())
                    {
                        ROS_INFO("[serviceUpdate] Sample ENU removed point: x=%.2f, y=%.2f, z=%.2f",
                                 removed_cloud_enu->points[0].x, removed_cloud_enu->points[0].y, removed_cloud_enu->points[0].z);
                    }
                    if (!storage_points_enu->empty())
                    {
                        ROS_INFO("[serviceUpdate] Sample ENU storage point: x=%.2f, y=%.2f, z=%.2f",
                                 storage_points_enu->points[0].x, storage_points_enu->points[0].y, storage_points_enu->points[0].z);
                    }

                    auto remove_start = std::chrono::high_resolution_clock::now();
                    vehicle::localization::utils::removePoints<PointT>(storage_points_enu, removed_cloud_enu); // Sử dụng removed_cloud_enu
                    auto remove_end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> remove_duration = remove_end - remove_start;
                    ROS_INFO("[serviceUpdate] Processed %zu points for removal in %.2f seconds",
                             removed_cloud_enu->size(), remove_duration.count());
                }
            }
        }

        if (req.points_added > 0 && !req.cloud_data.added_points.data.empty())
        {
            pcl::fromROSMsg(req.cloud_data.added_points, *added_cloud_mgrs);
            if (added_cloud_mgrs->empty())
            {
                ROS_WARN("[serviceUpdate] Added cloud is empty despite points_added = %lu", req.points_added);
            }
            else
            {
                *added_cloud_enu = *tf_pointcloud_enu(added_cloud_mgrs, added_cloud_enu);
                *storage_points_enu += *added_cloud_enu;

                ROS_INFO("[serviceUpdate] Added %zu new points to storage", added_cloud_mgrs->size());
                ROS_INFO("[serviceUpdate] Storage now contains %zu points", storage_points_enu->size());
            }
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*storage_points_enu, output);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "enu";
        point_enu_pub_.publish(output);
        // if (!init_)
        // {
        std::lock_guard<std::mutex> lock_target(monte_init_);
        storage_points_enu->header.frame_id = "enu";
        object_pre_->setInputTarget(storage_points_enu);
        ROS_INFO("setInputTarget success");
        init_ = true;
        // }
        vehicle_localization_util::SetLocalMap srv;
        if (!removed_cloud_enu->empty())
        {
            pcl::toROSMsg(*removed_cloud_enu, srv.request.removed_points_send);
            srv.request.removed_points_send.header.stamp = ros::Time::now();
            srv.request.removed_points_send.header.frame_id = "enu";
        }
        if (!added_cloud_enu->empty())
        {
            pcl::toROSMsg(*added_cloud_enu, srv.request.add_points_send);
            srv.request.add_points_send.header.stamp = ros::Time::now();
            srv.request.add_points_send.header.frame_id = "enu";
        }

        std::future<bool> set_local_map_future = std::async(std::launch::async, [this, &srv]()
                                                            { return send_local_map_client.call(srv); });
        if (set_local_map_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        {
            if (!set_local_map_future.get())
            {
                ROS_ERROR("[serviceUpdate] Send local map enu failed");
                res.success = false;
                return false;
            }
            else
            {
                ROS_INFO("[serviceUpdate] Send local map enu success");
                res.success = true;
            }
        }
        else
        {
            ROS_ERROR("[serviceUpdate] Send local map enu timed out");
            res.success = false;
            return false;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> load_duration = end_time - start_time;
        ROS_INFO("[serviceUpdate] Time to process serviceUpdate: %.2f seconds", load_duration.count());
        return true;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[serviceUpdate] Exception: %s", e.what());
        res.success = false;
        return false;
    }
}
bool Preprocessing::serviceInit(vehicle_localization_util::PoseWithCovarianceStamped::Request &req,
                                vehicle_localization_util::PoseWithCovarianceStamped::Response &res)
{
    std::lock_guard<std::mutex> lock(monte_init_);
    ROS_INFO("[serviceInit] Received pose: frame_id=%s, x=%.2f, y=%.2f, z=%.2f",
             req.pose_with_cov.header.frame_id.c_str(),
             req.pose_with_cov.pose.pose.position.x,
             req.pose_with_cov.pose.pose.position.y,
             req.pose_with_cov.pose.pose.position.z);
    geometry_msgs::TransformStamped transform;
    if (!vehicle::localization::utils::getTransform(
            tfBuffer, req.pose_with_cov.header.frame_id, "enu", transform))
    {
        ROS_ERROR("[serviceInit] Failed to get transform from %s to map",
                  req.pose_with_cov.header.frame_id.c_str());
        return false;
    }
    ROS_INFO("[serviceInit]Transform from %s to enu: trans_x=%.2f, trans_y=%.2f, trans_z=%.2f, rot_x=%.2f, rot_y=%.2f, rot_z=%.2f, rot_w=%.2f",
             req.pose_with_cov.header.frame_id.c_str(),
             transform.transform.translation.x,
             transform.transform.translation.y,
             transform.transform.translation.z,
             transform.transform.rotation.x,
             transform.transform.rotation.y,
             transform.transform.rotation.z,
             transform.transform.rotation.w);
    geometry_msgs::PoseWithCovarianceStamped map_tf_initial_pose;
    tf2::doTransform(req.pose_with_cov, map_tf_initial_pose, transform);
    ROS_INFO("[serviceInit] Initial pose: x=%.2f, y=%.2f, z=%.2f, frame_id=%s",
             map_tf_initial_pose.pose.pose.position.x,
             map_tf_initial_pose.pose.pose.position.y,
             map_tf_initial_pose.pose.pose.position.z,
             map_tf_initial_pose.header.frame_id.c_str());
    map_tf_initial_pose.header.frame_id = "enu";
    auto [pose_with_cov, score] = vehicle::localization::monte::alignUsingMonteCarloTPE(
        object_pre_, map_tf_initial_pose, params_, marker_pub_, points_monte_aligned_pub);
    if (!vehicle::localization::utils::isValidPose(pose_with_cov))
    {
        ROS_ERROR("[serviceInit] Monte Carlo alignment failed to produce valid result");
        return false;
    }
    if (score > 4.0)
    {
        init_monte_.publish(pose_with_cov);
        res.pose_with_cov = pose_with_cov;
        return true;
    }
    else
    {
        return false;
    }
}

void Preprocessing::points_callback(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{
    auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if (pcl_cloud->empty())
    {
        ROS_WARN("cloud is empty!!");
        return;
    }

    // Apply voxel grid filter first
    auto filtered = vehicle::localization::utils::voxelGridFilter<pcl::PointXYZI>(pcl_cloud, filter_point_);
    if (!filtered)
    {
        ROS_WARN("Voxel grid filtering failed!");
        return;
    }

    std::lock_guard<std::mutex> lock(monte_init_);

    // Check if transform is available
    if (tfBuffer.canTransform("base_link", points_msg->header.frame_id, ros::Time(0)))
    {
        try
        {
            // Get transform from lidar frame to base_link
            geometry_msgs::TransformStamped transform_stamped =
                tfBuffer.lookupTransform("base_link", points_msg->header.frame_id, ros::Time(0));

            // Manual conversion from TransformStamped to Eigen matrix
            Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
            /*Transform 4x4:
                 [R11  R12  R13  Tx]     R = Rotation matrix (3x3)
             T = [R21  R22  R23  Ty]     T = Translation vector (3x1)
                 [R31  R32  R33  Tz]     [0 0 0 1] = Homogeneous row
                 [ 0    0    0   1 ]
            */
            // Translation
            transform_matrix(0, 3) = transform_stamped.transform.translation.x;
            transform_matrix(1, 3) = transform_stamped.transform.translation.y;
            transform_matrix(2, 3) = transform_stamped.transform.translation.z;

            // Rotation (quaternion to rotation matrix)
            double x = transform_stamped.transform.rotation.x;
            double y = transform_stamped.transform.rotation.y;
            double z = transform_stamped.transform.rotation.z;
            double w = transform_stamped.transform.rotation.w;

            // Convert quaternion to rotation matrix
            transform_matrix(0, 0) = 1 - 2 * (y * y + z * z);
            transform_matrix(0, 1) = 2 * (x * y - w * z);
            transform_matrix(0, 2) = 2 * (x * z + w * y);

            transform_matrix(1, 0) = 2 * (x * y + w * z);
            transform_matrix(1, 1) = 1 - 2 * (x * x + z * z);
            transform_matrix(1, 2) = 2 * (y * z - w * x);

            transform_matrix(2, 0) = 2 * (x * z - w * y);
            transform_matrix(2, 1) = 2 * (y * z + w * x);
            transform_matrix(2, 2) = 1 - 2 * (x * x + y * y);

            // Transform point cloud to base_link frame
            pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());
            pcl::transformPointCloud(*filtered, *transformed_cloud, transform_matrix);

            // Update header frame_id
            transformed_cloud->header.frame_id = "base_link";

            // Use transformed cloud for Monte Carlo
            object_pre_->setInputSource(transformed_cloud);

            ROS_INFO_THROTTLE(5.0, "Point cloud transformed from %s to base_link. Size: %zu",
                              points_msg->header.frame_id.c_str(), transformed_cloud->size());
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform lookup failed: %s", ex.what());
            return;
        }
    }
    else
    {
        ROS_WARN_THROTTLE(5.0, "Cannot find transform from %s to base_link",
                          points_msg->header.frame_id.c_str());

        // Fallback: use original filtered cloud (not recommended for production)
        object_pre_->setInputSource(filtered);
    }
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Preprocessing::tf_pointcloud_enu(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &output)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    if (!input || input->empty())
    {
        ROS_ERROR("Input cloud is null or empty!");
        return output;
    }
    if (!output)
    {
        output.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    geometry_msgs::TransformStamped transform_map_enu;
    try
    {

        transform_map_enu = tfBuffer.lookupTransform("map", "enu", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("[tf_pointcloud_enu] %s", ex.what());
        ros::Duration(1.0).sleep();
        return output;
    }

    for (const auto &point : *input)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            ROS_WARN("Skipping invalid point: x=%f, y=%f, z=%f", point.x, point.y, point.z);
            continue;
        }

        geometry_msgs::PointStamped point_mgrs;
        point_mgrs.header.stamp = ros::Time::now();
        point_mgrs.header.frame_id = "map";
        point_mgrs.point.x = point.x;
        point_mgrs.point.y = point.y;
        point_mgrs.point.z = point.z;

        geometry_msgs::PointStamped point_enu;
        try
        {
            tf2::doTransform(point_mgrs, point_enu, transform_map_enu);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform failed for point: %s", ex.what());
            continue;
        }

        PointT enu_point;
        enu_point.x = point_enu.point.x;
        enu_point.y = point_enu.point.y;
        enu_point.z = point_enu.point.z;
        enu_point.intensity = point.intensity;
        if (!std::isfinite(enu_point.x) || !std::isfinite(enu_point.y) || !std::isfinite(enu_point.z))
        {
            ROS_WARN("Skipping invalid transformed point: x=%f, y=%f, z=%f", enu_point.x, enu_point.y, enu_point.z);
            continue;
        }
        output->points.push_back(enu_point);
    }

    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;

    if (!output->empty())
    {
        // int num_points_to_print = std::min(10, static_cast<int>(output->points.size()));
        // ROS_INFO("Printing first %d points from enu cloud:", num_points_to_print);
        // for (int i = 0; i < num_points_to_print; i++)
        // {
        //     const PointT &point = output->points[i];
        //     ROS_INFO("ENU Point %d: x=%.2f, y=%.2f, z=%.2f, intensity=%.2f",
        //              i, point.x, point.y, point.z, point.intensity);
        // }
    }
    else
    {
        ROS_WARN("Output cloud is empty!");
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> load_duration = end_time - start_time;
    ROS_INFO("[tf_pointcloud_enu] Time to process map enu : %.2f seconds",
             load_duration.count());
    return output;
}

void Preprocessing::callbackCreatePointAround(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO("Size of point enu %d", storage_points_enu->width);
    auto start_time = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> lock(map_push);
    float current_x = msg->pose.pose.position.x;
    float current_y = msg->pose.pose.position.y;
    float current_z = msg->pose.pose.position.z;
    if (storage_points_enu->empty())
    {
        ROS_ERROR("[callbackMapSubRaw] Received empty PointCloud data on first load!");
        return;
    }

    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(storage_points_enu);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);
    // ROS_INFO("Size of point enu after voxel %d", filtered->width);
    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(current_x - radius_max_, current_x + radius_max_);
    pass_x.filter(*storage_points_around);
    // ROS_INFO("After x-pass filter: %zu points", storage_points_around->size());

    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(storage_points_around);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(current_y - radius_max_, current_y + radius_max_);
    pass_y.filter(*storage_points_around);
    // ROS_INFO("After y-pass filter: %zu points", storage_points_around->size());

    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(storage_points_around);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(current_z - radius_max_, current_z + radius_max_);
    pass_z.filter(*storage_points_around);
    // ROS_INFO("After z-pass filter: %zu points", storage_points_around->size());

    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*storage_points_around, filtered_msg);
    filtered_msg.header.frame_id = "enu";
    filtered_msg.header.stamp = ros::Time::now();
    point_enu_around.publish(filtered_msg);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_passthrough = end_time - start_time;
    // ROS_INFO("[loadMapAsync] Time to passthroug: %.2f seconds", time_passthrough.count());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "preprocessing");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    Preprocessing preprocessing(nh, private_nh);
    ROS_INFO("[Preprocessing] Ready to Preprocessing.");

    ros::spin();
    return 0;
}