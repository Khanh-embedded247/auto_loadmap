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
#ifndef AUTO_LOAD_H
#define AUTO_LOAD_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <future>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <unordered_set>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
// Filter pointcloud by sphere radius
//  #include <pcl/filters/crop_sphere.h>
#include <pcl/pcl_config.h>
#include <vector>
#include <algorithm>
#include <map>
#include <tf/transform_broadcaster.h>
#include "vehicle_localization_util/updatePoints.h"
#include "vehicle_localization_util/pnkx_utils.h"

class GlobalLocalizationTestNode;
/**
 * @struct AreaInfo
 * @brief Stores information about an area, including name, path, and
 * boundaries.
 */
struct AreaInfo
{
    std::string area_name;        ///< Name of the area
    std::string full_path;        ///< Full path to the area configuration
    float xmin, ymin, xmax, ymax; ///< Area boundaries
};

/**
 * @struct MapInfo
 * @brief Represents metadata for the map, including map name, boundaries, map
 * status, and cloud data.
 */
struct MapInfo
{
    std::string filename;                       ///< Name of the map file
    float xmin, ymin, xmax, ymax;               ///< Map boundaries
    bool is_loaded = false;                     ///< Indicates if the map is loaded
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; ///< Point cloud data of the map
    bool is_current = false;                    ///< Indicates if this is the current map
};

class MapAutoLoad
{
public:
    using PointT = pcl::PointXYZI;
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param private_nh Private ROS node handle
     */
    MapAutoLoad(ros::NodeHandle nh, ros::NodeHandle private_nh);

    /**
     * @brief Destructor
     */
    ~MapAutoLoad();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher point_filter_pub_;
    ros::Publisher init_pose_MGRS_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher tf_pub_;

    ros::Subscriber gnss_pose_sub_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber point_ref_sub_;
    ros::ServiceClient points_update_client_;

    ros::Timer filter_timer_;
    // ros::Timer tf_broadcast_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Configuration file path
    std::string base_pcd_path_;
    std::string map_config_path_;

    std::string current_area_; ///< Name of the current area

    std::vector<MapInfo> maps_to_load;   ///< List of maps to be loaded
    std::vector<MapInfo> maps_to_unload; ///< List of maps to be unloaded

    // Containers contain map data
    pcl::PointCloud<PointT>::Ptr
        current_merged_cloud_; ///< Merged point cloud data of all loaded maps
    pcl::PointCloud<PointT>::Ptr current_merged_cloud_filter_;
    ;
    std::vector<MapInfo> available_maps_; ///<///< List of all available maps
                                          ///<///< List of all available maps
    std::map<std::string, pcl::PointCloud<PointT>::Ptr>
        loaded_clouds_; ///< Map of loaded maps and their point clouds

    std::queue<std::pair<float, float>>
        position_queue_;     ///< Queue of vehicle positions
    std::mutex queue_mutex_; ///< Mutex for position queue
    std::condition_variable map_update_cv_;
    std::thread map_update_thread_; ///< Thread for map updates
    std::atomic<bool> should_stop_; ///< Flag for stopping the map update thread
    // Mutexes for thread safety
    std::mutex merge_mutex_;
    std::mutex cloud_mutex_;
    std::mutex map_state_mutex_;
    std::mutex gnss_mutex_;
    std::mutex points_mutex_;
    
    std::chrono::high_resolution_clock::time_point start_time_gnss;
    bool gnss_received_ = false;
    bool loaded_boundaries_file_ = false; ///< Indicates if boundaries file is loaded
    bool pub_tf_done_ = false;
    bool first_point_ = false;
    bool origin_set_ = false;
    size_t total_points_added = 0;
    size_t total_points_removed = 0;
    double origin_lat_, origin_lon_, origin_height_;

    geometry_msgs::TransformStamped transform_map_enu_;
    std::mutex mtx_tf;
    pcl::PointCloud<pcl::PointXYZI>::Ptr added_points_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr removed_points_cloud;
    geometry_msgs::PoseWithCovarianceStamped gnss_sub_data_;

    // pcl::KdTreeFLANN<PointT> kdtree_;
    GeographicLib::LocalCartesian local_cartesian_;

    void setupPublishers();
    void setupSubscribers();
    void setupServices();

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
     * @brief Generates a YAML configuration file for a specific area.
     *
     * Creates a YAML file based on the information of the given area's directory.
     * This method is useful for dynamically setting up configurations for new
     * areas.
     *
     * @param area_path Path to the directory where the YAML file will be
     * generated
     */
    void generateYamlForArea(const std::string &input_pcd);

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

    // /**
    //  * @brief Generates a YAML configuration file for a specific area.
    //  *
    //  * Creates a YAML file based on the information of the given area's
    //  directory.
    //  * This method is useful for dynamically setting up configurations for new
    //  areas.
    //  *
    //  * @param area_path Path to the directory where the YAML file will be
    //  generated
    //  */
    // void generateYamlForArea(const std::string &area_path);

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
     * @brief Filters a point cloud using a voxel grid.
     * @param cloud Pointer to the input point cloud
     * @return Pointer to the filtered point cloud
     */
    pcl::PointCloud<PointT>::Ptr filterWithVoxelGrid(
        pcl::PointCloud<PointT>::Ptr cloud);

    /**
     * @brief Loads map configurations from a YAML file.
     * @param map_config_path_ Path to the configuration YAML file
     */
    void loadMapConfigurations(const std::string &map_config_path_);

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
    float pointToLineDistance(float px, float py, float x1, float y1, float x2,
                              float y2);

    /**
     * @brief Calculates the distance from a given position to a map boundary.
     * @param x X-coordinate of the position
     * @param y Y-coordinate of the position
     * @param map Reference to the MapInfo structure
     * @return Distance from the position to the map boundary
     */
    float calculateDistanceToMap(float x, float y, const MapInfo &map);

    /**
       @brief Continuously updates the map state based on the vehicle's current
       location Processing when the vehicle is stationary but the callback keeps
       running, reducing processing operations
       */
    void startMapUpdateThread();

    /**
     * @brief Determines whether a map is required based on the vehicle's
     * position.
     * @param x X-coordinate of the vehicle
     * @param y Y-coordinate of the vehicle
     * @param map Reference to the MapInfo structure
     * @return True if the map is needed, false otherwise
     */
    bool isMapNeeded(float x, float y, const MapInfo &map);

    /**
     * @brief Processes map updates based on the vehicle's position.
     * @param current_x X-coordinate of the vehicle's position
     * @param current_y Y-coordinate of the vehicle's position
     */
    void processMapUpdate(float current_x, float current_y);

    /**
     * @brief Unloads a map asynchronously.
     * @param filename Name of the map file to unload
     */
    void unloadMapAsync(const std::string &filename);

    /**
     * @brief Loads a map asynchronously.
     * @param filename Name of the map file to load
     */
    void loadMapAsync(const std::string &filename);

    /**
     * @brief Updates the merged point cloud by adding and removing specified
     * maps.
     * @param maps_to_load List of map filenames to be loaded
     * @param maps_to_unload List of map filenames to be unloaded
     */
    void updateMergedCloud(const std::vector<std::string> &maps_to_load,
                           const std::vector<std::string> &maps_to_unload);

    /**
     * @brief Retrieves the corresponding OSM file for a given PCD file.
     * @param pcd_file Name of the PCD file
     * @return Path to the associated OSM file
     */
    std::string getOSMFile(const std::string &pcd_file);

    /**
     * @brief Callback for vehicle pose updates.
     * @param msg Pointer to the PoseStamped message
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void pointRefCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    /**
     * @brief Publish cloud data of current map
     */

    // void TfBroadcastPub(const ros::TimerEvent &);

    void filterMapData(const ros::TimerEvent &);
    /**
     * @brief Callback for processing GNSS pose with covariance data.
     *
     * @param[in] pose_cov_msg_ptr A pointer to the PoseWithCovarianceStamped
     * message.
     */
    
    void callbackGNSSPoseCov(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr
            &pose_cov_msg_ptr);
    double getGroundHeight(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcdmap,
        const tf2::Vector3 &point);
    bool notifyPointsUpdate(size_t points_added, size_t points_removed, const pcl::PointCloud<pcl::PointXYZI>::Ptr &added_cloud,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &removed_cloud);
};

#endif // AUTO_LOAD_H
