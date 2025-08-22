/*
* This file is part of an automatic map loading task.
*
* Developed for Phenikaa Joint Stock Company's Autonomous Self-Driving Car
*System. This product includes software developed by engineer Dang Dinh Khanh
*See the COPYRIGHT File section in the top-level directory of this distribution
for details about code ownership.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See

* GNU General Public License for more details.
*
* You will receive a copy of the GNU General Public License
* along with this program. The program was written and developed by engineer
Dang Dinh Khanh. *For more details contact:https://github.com/Khanh-embedded247
*/
#pragma once

#ifndef AUTO_LOAD_H
#define AUTO_LOAD_H
// Standard Library
#include <future>
#include <memory>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>

// Third-party Libraries
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

// ================= ROS =================
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

// ================= PCL =================
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// ================= TF =================
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ================= GeographicLib =================
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

// ================= Project =================
#include "vehicle_localization_util/updatePoints.h"
#include "basic_process_operations/func_base.h"
#include "vehicle_localization_util/PoseWithCovarianceStamped.h"

namespace fs = boost::filesystem;
/**
 * @struct Boundaries
 * @brief Defines the rectangular boundaries of a map area.
 */
struct Boundaries
{
    float x_min, y_min, x_max, y_max;
};

/**
 * @struct AreaInfo
 * @brief Stores information about a specific map area.
 */
struct AreaInfo
{
    std::string area_name; ///< Name of the area
    std::string full_path; ///< Full path to the area configuration
    Boundaries boundaries; ///< Area boundaries
    int pointCount;
    bool isLoaded;
    bool isPublish;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

/**
 * @class AutoLoadMapSystem
 * @brief Manages automatic loading/unloading of map data based on vehicle position.
 */
class AutoLoadMapSystem
{
public:
    using PointT = pcl::PointXYZI;

    AutoLoadMapSystem(ros::NodeHandle nh, ros::NodeHandle private_nh);

    /**
     * @brief Destructor
     */
    ~AutoLoadMapSystem();

private:
    /**
     * @brief Constructor
     * @param nh_ ROS node handle
     * @param private_nh_ Private ROS node handle
     */
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    /// **********PUBLISHERS*******************
    ros::Publisher init_pose_MGRS_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher tf_pub_;

    /// **********SUBSCRIBERS*******************
    ros::Subscriber gnss_pose_sub_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber point_ref_sub_;
    ros::Subscriber initialpose_sub_;

    ///***********SERVICE CLIENT****************
    ros::ServiceClient points_update_client_;
    ros::ServiceClient first_point_client;

    // === Timer ===
    ros::Timer filter_timer_;

    // === TF ===
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // === GeographicLib ===
    GeographicLib::LocalCartesian local_cartesian_;

    // === Map State ===
    std::vector<AreaInfo> available_maps_;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr> loaded_clouds_;
    pcl::PointCloud<PointT>::Ptr current_merged_cloud_; ///< Merged point cloud data of all loaded maps
    pcl::PointCloud<pcl::PointXYZI>::Ptr added_points_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr removed_points_cloud;

    // === Queues ===
    std::queue<std::pair<float, float>> position_queue_;
    std::queue<geometry_msgs::PoseWithCovarianceStamped> gnss_queue;

    // === Config ===
    std::string map_config_path_;
    std::string base_pcd_path_;
    std::string current_area_;

    // === State Flags ===
    bool origin_set_ = false;
    bool first_point_ = false;
    bool pub_tf_done_ = false;
    bool loaded_boundaries_file_ = false;
    bool has_map = false;
    bool processing_gnss = false;
    bool initialpose_received_ = false;

    // === Position & Origin ===
    float current_x = 0.0f, current_y = 0.0f;
    double origin_lat_ = 0.0, origin_lon_ = 0.0, origin_height_ = 0.0;

    // === Statistics ===
    size_t total_points_added = 0;
    size_t total_points_removed = 0;

    // === Transforms ===
    geometry_msgs::TransformStamped transform_map_enu_;
    geometry_msgs::PoseWithCovarianceStamped gnss_sub_data_;
    geometry_msgs::PoseWithCovarianceStamped resuilt_init_;

    // === Threading ===
    std::thread map_update_thread_;
    std::atomic<bool> should_stop_;
    std::mutex queue_mutex_;
    std::mutex map_state_mutex_;
    std::mutex cloud_mutex_;
    std::mutex merge_mutex_;
    std::mutex mtx_tf;
    std::mutex gnss_mutex;
    std::condition_variable map_update_cv_;

    // ============ Setup ============
    void setupPublishers();
    void setupSubscribers();
    void setupServices();

    /// ================================= Map Handling ===========================
    /**
     * @brief Check if the map boundary has been extracted. If not created.
     * @param path_to_pcd path containing boundary file
     */
    bool checkBoundariesFile(const std::string &path_to_pcd);

    /**
     * @brief Create boundaries file from map pcd .
     * @param input_pcd path containing map pcd
     */
    void generateYamlForArea(const std::string &input_pcd);

    /**
     * @brief Get the first data to do pose_init in MGRS coordinates
     * @param[in] pose_cov_msg_ptr first GNSS data after conversion to MGRS coordinates
     */

    /**
     * @brief Loads map configurations from a YAML file.
     * @param map_config_path_ Path to the configuration YAML file
     */
    void loadMapConfigurations(const std::string &map_config_path_);

    /**
     * @brief Loads and updates the map data for a specified area.
     *
     * This method manages loading of map point cloud data and updates the
     * current map state when the vehicle enters a new area. It ensures that
     * only relevant map data is loaded to optimize performance.
     *
     * @param area_info Structure containing details of the area to be updated
     * @return True if the map was successfully loaded and updated, false
     * otherwise
     */
    bool loadAndUpdateArea(const AreaInfo &area_info);

    /**
     * @brief Finds and loads YAML configuration for a specific area.
     *
     * This method searches for a YAML configuration file in the given
     * area path and loads the configuration details to update internal states.
     *
     * @param area_path Path to the directory containing the area's configuration
     * file
     */
    void findYamlConfig(const std::string &area_path);

    /**
     * @brief Processes map updates based on the vehicle's position.
     * @param current_x X-coordinate of the vehicle's position
     * @param current_y Y-coordinate of the vehicle's position
     */
    void processMapUpdate(float current_x, float current_y);

    /**
     * @brief Updates the merged point cloud by adding and removing specified
     * maps.
     * @param maps_to_load List of map filenames to be loaded
     * @param maps_to_unload List of map filenames to be unloaded
     */
    void updateMergedCloud(const std::vector<std::string> &maps_to_publish);

    /**
     * @brief Loads a map asynchronously.
     * @param filename Name of the map file to load
     */
    void loadMapAsync(const std::string &filename);

    /**
     * @brief Unloads a map asynchronously.
     * @param filename Name of the map file to unload
     */
    void unloadMapAsync(const std::string &filename);

    /**
     * @brief Filters a point cloud using a voxel grid.
     * @param cloud Pointer to the input point cloud
     * @return Pointer to the filtered point cloud
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterWithVoxelGrid(pcl::PointCloud<PointT>::Ptr cloud);

    ///==================================================================

    // ============ GNSS & Pose ============
    /**
     * @brief Callback for GNSS pose with covariance messages.
     *
     * - Publishes initial pose in MGRS if not yet initialized.
     * - Buffers incoming GNSS poses in a queue (max size 2).
     * - Checks time synchronization and filters out invalid/noisy data.
     * - Processes GNSS data when vehicle has moved within a 3m threshold.
     * - Sets initial pose flag after successful convergence.
     */
    void callbackGNSSPoseCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_cov_msg_ptr);

    /**
     @brief Continuously updates the map state based on the vehicle's current
     location Processing when the vehicle is stationary but the callback keeps
     running, reducing processing operations
     */
    bool processGNSSData(
        geometry_msgs::PoseWithCovarianceStamped &pose_cov_msg);

    /**
     * @brief Call the NDT alignment initialization service.
     *
     * - Sends GNSS-based initial pose to NDT align server.
     * - Updates output pose with server response.
     * - Returns true on success, false otherwise.
     */
    bool callInitService(
        const geometry_msgs::PoseWithCovarianceStamped &input_pose_msg,
        const geometry_msgs::PoseWithCovarianceStamped::Ptr &output_pose_msg_ptr);

    /**
     * @brief Get the first Ublox data to do the coordinate system transformation reference
     */
    void pointRefCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    /**
     * @brief Callback for vehicle pose updates.
     * @param msg Pointer to the PoseStamped message
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    ///==================================================================

    // ============ Area Detection ============

    /**
     * @brief Identify the current vehicle area based on its position.
     *
     * This method checks the vehicle's current position (x, y) and determines
     * which area it belongs to by comparing it with predefined area boundaries.
     *
     * @param x X-coordinate of the vehicle's position
     * @param y Y-coordinate of the vehicle's position
     * @return AreaInfo structure containing details of the identified area
     */
    AreaInfo detectCurrentArea(float x, float y);

    /**
     * @brief Check if the vehicle location is on the map
     */
    bool isVehicleInMap(float x, float y, const AreaInfo &map);

    /**
     * @brief Check if a map should be loaded based on current position.
     *
     * - Calculates distance from (x,y) to the given map area.
     * - Returns true if within 50 meters.
     */
    bool isMapNeeded(float x, float y, const AreaInfo &map, float &distance);

    /**
     * @brief Calculates the distance from a given position to a map boundary.
     * @param x X-coordinate of the position
     * @param y Y-coordinate of the position
     * @param map Reference to the MapInfo structure
     * @return Distance from the position to the map boundary
     */
    ///===========================================================

    // ============ Geometry ============
    float calculateDistanceToMap(float x, float y,
                                 const AreaInfo &map);

    /**
     * @brief Computes the distance from a point to a line segment.
     * @param px X-coordinate of the point
     * @param py Y-coordinate of the point
     * @param x1 X-coordinate of the first endpoint of the line
     * @param y1 Y-coordinate of the first endpoint of the line
     * @param x2 X-coordinate of the second endpoint of the line
     * @param y2 Y-coordinate of the second endpoint of the line
     * @return The perpendicular distance from the point to the line segment
     */
    float pointToLineDistance(float px, float py, float x1, float y1,
                              float x2, float y2);

    /**
     * @brief Get the ground height at a given point on the map.
     *
     * @param[in] pcdmap A pointer to the point cloud map.
     * @param[in] point The point where the height is calculated.
     *
     * @return The ground height at the given point.
     */
    double getGroundHeight(
        const pcl::PointCloud<PointT>::Ptr &pcdmap,
        const tf2::Vector3 &point);
    //==============================================================

    // ============ Updates ============
    /**
     * @brief Starts a background thread to update maps based on position queue.
     *
     * - Waits for new positions pushed into queue.
     * - Processes map update for each position (FIFO).
     * - Stops gracefully when should_stop_ is set.
     */
    void startMapUpdateThread();

    /**
     * @brief Publish cloud data of current map
     */
    void filterMapData(const ros::TimerEvent &);

    /**
     * @brief Service Client sends data point add or delete request to update input
     */
    bool notifyPointsUpdate(size_t points_added, size_t points_removed,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &added_cloud,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &removed_cloud);
    //==================================================================
};
#endif // AUTO_LOAD_H