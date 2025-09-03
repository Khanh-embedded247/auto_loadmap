#include "auto_loadmap/pre_processing.h"

Preprocessing::Preprocessing(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh),
      storage_points_around(new pcl::PointCloud<pcl::PointXYZI>),
      storage_points_enu(new pcl::PointCloud<pcl::PointXYZI>), map_frame("enu"),
 tfListener(tfBuffer)
{
    setupPublishers();
    setupSubscribers();
    setupServices();
    private_nh_.param<float>("radius_max", radius_max_, 30.0);

    private_nh_.param<double>("downsample_resolution", downsample_resolution, 0.3);
}
Preprocessing::~Preprocessing() {};
void Preprocessing::setupPublishers()
{
    point_enu_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_enu", 10);
    point_enu_around = nh_.advertise<sensor_msgs::PointCloud2>("/map_enu_around", 10);
}
void Preprocessing::setupSubscribers()
{
    odom_enu_sub_ = nh_.subscribe("/odom_align", 1, &Preprocessing::callbackCreatePointAround, this);
}
void Preprocessing::setupServices()
{
    points_update_server_ = nh_.advertiseService("points_update_service", &Preprocessing::serviceUpdate, this);
    send_local_map_client = nh_.serviceClient<vehicle_localization_util::SetLocalMap>("map_enu_update");
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
        storage_points_enu->header.frame_id = "enu";

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