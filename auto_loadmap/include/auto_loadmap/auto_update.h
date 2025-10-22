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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <diagnostic_updater/diagnostic_updater.h>
// #include <GeographicLib/LocalCartesian.hpp>
// #include <vehicle_localization_util/updatePoints.h>
// #include <vehicle_localization_util/PoseWithCovarianceStamped.h>

namespace fs = boost::filesystem;

using PointT = pcl::PointXYZI;

struct Boundaries {
    float x_min, y_min, z_min;
    float x_max, y_max, z_max;
};

struct MapInfo {
    std::string map_name;   // e.g., hungyen_a1.pcd
    std::string full_path;  // Full path to .pcd file
    Boundaries boundaries;  // 3D boundaries (x,y,z)
    bool isLoaded = false;
    bool isPublish = false;
    pcl::PointCloud<PointT>::Ptr cloud;
    int pointCount = 0;  // Number of points in the map
};

struct AreaInfo {
    std::string area_name;  // e.g., Area_A
    std::string full_path;  // Full path to area directory
    Boundaries boundaries;  // 3D boundaries (x,y,z)
    std::vector<MapInfo> maps;  // Maps in this area
    bool isLoaded = false;
};

struct DistrictInfo {
    std::string district_name;  // e.g., Hungyen
    std::string full_path;     // Full path to district directory
    std::array<float, 2> min_xy;  // 2D min (x,y) for district
    std::array<float, 2> max_xy;  // 2D max (x,y) for district
    std::vector<AreaInfo> areas;  // Areas in this district
};

struct DiagnosticLoadmap{
    int point_map_currents_;
    std::string address_;
    bool boundaries_area_;
    bool boundaries_map_;
    bool boundaries_district_;
    std::vector<std::string> map_current_;
}diag_loadmap_;
class AutoLoadMapSystem {
public:
    AutoLoadMapSystem(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~AutoLoadMapSystem();

private:
    // ROS Node Handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Publishers
    ros::Publisher point_cloud_pub_;
    ros::Publisher init_pose_MGRS_;
    ros::Timer filter_timer_;

    // Subscribers
    ros::Subscriber pose_subscriber_;
    ros::Subscriber current_position_subscriber_;

    //Timer
    ros::Timer timer_Loadmap;

    // Point Clouds
    pcl::PointCloud<PointT>::Ptr added_points_cloud;
    pcl::PointCloud<PointT>::Ptr removed_points_cloud;
    pcl::PointCloud<PointT>::Ptr current_merged_cloud_;
    std::unordered_map<std::string, pcl::PointCloud<PointT>::Ptr> loaded_clouds_;
    size_t total_points_added = 0;
    size_t total_points_removed = 0;

    // Threading
    std::thread map_update_thread_;
    std::mutex cloud_mutex_;
    std::mutex map_state_mutex_;
    std::mutex merge_mutex_;
    std::mutex gnss_mutex;
    std::mutex queue_mutex_;
    std::mutex mtx_tf;
    std::condition_variable map_update_cv_;
    bool should_stop_;

    // GNSS and Positioning
    std::queue<geometry_msgs::PoseWithCovarianceStamped> gnss_queue;
    geometry_msgs::PoseWithCovarianceStamped gnss_sub_data_;
    geometry_msgs::PoseWithCovarianceStamped resuilt_init_;
    bool processing_gnss = false;
    bool initialpose_received_ = false;
    bool has_map = false;

    // Map Management
    std::string map_config_path_;
    std::string base_pcd_path_;
    // std::string current_district_;
    std::string current_area_;
    bool loaded_boundaries_file_ = false;
    std::vector<MapInfo> available_maps_;
    std::queue<std::pair<float, float>> position_queue_;

    diagnostic_updater::Updater updater_loadmap_;

    // Methods
    bool checkBoundariesFile(const std::string &path_to_pcd);
    void generateYamlForMap(const std::string &area_path);
    void generateYamlForArea(const std::string &district_path);
    void setupPublishers();
    void setupSubscribers();

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void startMapUpdateThread();
    DistrictInfo detectCurrentDistrict(float x, float y);
    AreaInfo detectCurrentArea(const DistrictInfo &district, float x, float y);
    MapInfo detectCurrentMap(const AreaInfo &area, float x, float y);
    void findYamlConfig(const std::string &district_path);
    void loadMapConfigurations(const std::string &district_boundaries_path);
    bool loadAndUpdateArea(const AreaInfo &area_info);
    bool isVehicleInMap(float x, float y, const MapInfo &map);
    bool isMapNeeded(float x, float y, const MapInfo &map, float &distance);
    float calculateDistanceToMap(float x, float y, const MapInfo &map);
    float pointToLineDistance(float px, float py, float x1, float y1, float x2, float y2);
    pcl::PointCloud<PointT>::Ptr filterWithVoxelGrid(pcl::PointCloud<PointT>::Ptr cloud);
    void processMapUpdate(float current_x, float current_y);
    void loadMapAsync(const std::string &filename);
    void unloadMapAsync(const std::string &filename);
    void updateMergedCloud(const std::vector<std::string> &maps_to_publish);
    void filterMapData(const ros::TimerEvent &);
    void timer_diag_loadmap();
    void checkAutoLoadmapStatus(diagnostic_updater::DiagnosticStatusWrapper &status);
};
#endif // AUTO_LOAD_H