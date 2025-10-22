#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <ros/ros.h>
#include <mutex>
#include <map>
#include <future>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf/transform_broadcaster.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <GeographicLib/LocalCartesian.hpp>
// #include <GeographicLib/MGRS.hpp>
// #include <GeographicLib/UTMUPS.hpp>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <chrono>

class Preprocessing
{
    using PointT = pcl::PointXYZI;

public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param private_nh Private ROS node handle
     */
    Preprocessing(ros::NodeHandle nh, ros::NodeHandle private_nh);

    /**
     * @brief Destructor
     */
    ~Preprocessing();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher point_around;

    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;
    std::mutex map_push;
    std::mutex cloud_mutex_;

    // geometry_msgs::TransformStamped transform_map_enu_;
    pcl::PointCloud<PointT>::Ptr storage_points_around;
    pcl::PointCloud<PointT>::Ptr storage_points_;

    std::string map_frame;
    float radius_max_;
    double downsample_resolution;

    void setupPublishers();
    void setupSubscribers();
void callbackMapPoints(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void callbackCreatePointAround(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

#endif // PREPROCESSING_H