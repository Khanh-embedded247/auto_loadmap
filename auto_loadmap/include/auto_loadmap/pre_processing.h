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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include "vehicle_localization_util/updatePoints.h"
#include "vehicle_localization_util/TypeDataUpdatePoint.h"
#include "basic_process_operations/func_base.h"
#include "vehicle_localization_util/PoseWithCovarianceStamped.h"
#include "vehicle_localization_util/SetLocalMap.h"

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

    ros::Publisher point_enu_pub_;
    ros::Publisher point_enu_around;

    ros::Subscriber odom_enu_sub_;

    ros::ServiceServer points_update_server_;
    ros::ServiceClient send_local_map_client;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::mutex map_push;
    std::mutex cloud_mutex_;

    // geometry_msgs::TransformStamped transform_map_enu_;
    pcl::PointCloud<PointT>::Ptr storage_points_around;
    pcl::PointCloud<PointT>::Ptr storage_points_enu;



    std::string map_frame;
    float radius_max_;
    double downsample_resolution;

    void setupPublishers();
    void setupSubscribers();
    void setupServices();
    void callbackCreatePointAround(const nav_msgs::Odometry::ConstPtr &msg);
    bool serviceUpdate(
        vehicle_localization_util::updatePoints::Request &req,
        vehicle_localization_util::updatePoints::Response &res);
  
    pcl::PointCloud<PointT>::Ptr tf_pointcloud_enu(pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                                                   pcl::PointCloud<pcl::PointXYZI>::Ptr &output);
};

#endif // PREPROCESSING_H