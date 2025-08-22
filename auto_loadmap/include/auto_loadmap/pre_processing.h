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
#include <geometry_msgs/TransformStamped.h>
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
#include "basic_process_operations/monte_carlo.h"
#include "vehicle_localization_util/PoseWithCovarianceStamped.h"
#include "vehicle_localization_util/SetLocalMap.h"

#include "pclomp/ndt_omp.h"
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
    ros::Publisher points_monte_aligned_pub;
    ros::Publisher marker_pub_;
    ros::Publisher init_monte_;

    ros::Subscriber odom_enu_sub_;
    ros::Subscriber points_sub;

    ros::ServiceServer points_update_server_;
    ros::ServiceServer first_point_server;
    ros::ServiceClient send_local_map_client;
    ros::ServiceClient first_point_client;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::mutex map_push;
    std::mutex cloud_mutex_;
    std::mutex monte_init_;
    bool received_gnss_ = false;

    // geometry_msgs::TransformStamped transform_map_enu_;
    pcl::PointCloud<PointT>::Ptr storage_points_around;
    pcl::PointCloud<PointT>::Ptr storage_points_enu;
    pcl::PointCloud<PointT>::Ptr storage_points_MGRS;
    GeographicLib::LocalCartesian local_cartesian_;
    boost::shared_ptr<pcl::PointCloud<PointT>> transformed_cloud;

    std::string map_frame;
    int temp_n_startup_trials;
    int temp_particles_num;
    float filter_point_;

    float radius_max_;
    double downsample_resolution;
    
    bool received_point_raw_ = false;
    bool firt_gnss_ = false;
    bool init_ = false;
    
    std::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> object_pre_;
    vehicle::localization::monte::Parameters params_;

    void setupPublishers();
    void setupSubscribers();
    void setupServices();
    void callbackCreatePointAround(const nav_msgs::Odometry::ConstPtr &msg);
    bool serviceInit(vehicle_localization_util::PoseWithCovarianceStamped::Request &req,
                     vehicle_localization_util::PoseWithCovarianceStamped::Response &res);
    void points_callback(const sensor_msgs::PointCloud2ConstPtr &points_msg);
    bool serviceUpdate(
        vehicle_localization_util::updatePoints::Request &req,
        vehicle_localization_util::updatePoints::Response &res);
    // void removePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr storage_points_raw,
    //                   pcl::PointCloud<pcl::PointXYZI>::Ptr removed_cloud);
    pcl::PointCloud<PointT>::Ptr tf_pointcloud_enu(pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                                                   pcl::PointCloud<pcl::PointXYZI>::Ptr &output);
};

#endif // PREPROCESSING_H