#include "auto_loadmap/pre_processing.h"

Preprocessing::Preprocessing(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh), storage_points_(new pcl::PointCloud<pcl::PointXYZI>),
      storage_points_around(new pcl::PointCloud<pcl::PointXYZI>)
{
    setupPublishers();
    setupSubscribers();

    private_nh_.param<float>("radius_max", radius_max_, 30.0);

    private_nh_.param<double>("downsample_resolution", downsample_resolution, 0.3);
}
Preprocessing::~Preprocessing() {};
void Preprocessing::setupPublishers()
{
    point_around = nh_.advertise<sensor_msgs::PointCloud2>("/map_enu_around", 10);
}
void Preprocessing::setupSubscribers()
{
    odom_sub_ = nh_.subscribe("/localization/pose_twist_fusion_filter/pose", 1, &Preprocessing::callbackCreatePointAround, this);
    // map_sub_ = nh_.subscribe("/map/pointcloud_map", 1, &Preprocessing::callbackMapPoints, this);
}
void Preprocessing::callbackMapPoints(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    ROS_INFO("[Preprocessing] RECEIVED DATA MAP");
    pcl::fromROSMsg(*msg, *storage_points_);
    if (storage_points_->empty())
    {
        ROS_ERROR("cloud is empty, skipping frame...");
        return;
    }
    std::cout << "[Preprocessing] Number points: " << storage_points_->size() << std::endl;
}
void Preprocessing::callbackCreatePointAround(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // ROS_INFO("Size of point enu %d", storage_points_enu->width);
    auto start_time = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> lock(map_push);
    float current_x = msg->pose.position.x;
    float current_y = msg->pose.position.y;
    float current_z = msg->pose.position.z;
    std::cout << "Number points in current map : " << storage_points_->size() << std::endl;
    if (storage_points_->empty())
    {
        ROS_WARN("[callbackMapSubRaw] Received empty PointCloud data on first load!");
        return;
    }

    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(storage_points_);
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
    point_around.publish(filtered_msg);
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