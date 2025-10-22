#include "auto_loadmap/auto_update.h"

AutoLoadMapSystem::AutoLoadMapSystem(
  ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh),
  private_nh_(private_nh),
  added_points_cloud(new pcl::PointCloud<pcl::PointXYZI>),
  removed_points_cloud(new pcl::PointCloud<pcl::PointXYZI>),
  current_merged_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
  should_stop_(false)
{
  private_nh_.param<std::string>(
    "map_config_path", map_config_path_,
    "/root/phenikaax_pilot/src/pnkx_core/pnkx_localization/auto_loadmap/map/"
    "district");
  private_nh_.param<std::string>(
    "base_pcd_path", base_pcd_path_,
    "/root/phenikaax_pilot/src/pnkx_core/pnkx_localization/auto_loadmap/map/"
    "district");

  ROS_INFO("[AutoLoadMapSystem] Node initialized.");
  bool check_boundaries = checkBoundariesFile(base_pcd_path_);
  if (check_boundaries) {
    ROS_INFO("[checkBoundariesFile] Checked boundaries for all areas.");
  }
  setupPublishers();
  setupSubscribers();

updater_loadmap_.setHardwareID("Auto Loadmap for Localization");
  updater_loadmap_.add(
    "AutoLoadmap Status", this, &AutoLoadMapSystem::checkAutoLoadmapStatus);
  updater_loadmap_.add(
    "heartbeat", [](diagnostic_updater::DiagnosticStatusWrapper & s) {
      s.summary(diagnostic_msgs::DiagnosticStatus::OK, "Alive");
    });
  timer_Loadmap = nh_.createTimer(
    ros::Duration(0.1), std::bind(&AutoLoadMapSystem::timer_diag_loadmap, this));
  // setupServices();
  startMapUpdateThread();
}

AutoLoadMapSystem::~AutoLoadMapSystem()
{
  should_stop_ = true;
  map_update_cv_.notify_all();
  if (map_update_thread_.joinable()) { map_update_thread_.join(); }
}

void AutoLoadMapSystem::timer_diag_loadmap(){
  updater_loadmap_.force_update();
}

void AutoLoadMapSystem::checkAutoLoadmapStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::DiagnosticStatus;
  int overall_level = DiagnosticStatus::OK;
  std::vector<std::string> parts;

  // point map
  if (diag_loadmap_.point_map_currents_ == 0) {
    overall_level
      = std::max(overall_level, static_cast<int>(DiagnosticStatus::ERROR));
    parts.emplace_back("No maps have been published yet.");
  } else {
    parts.emplace_back("Map published");
  }

  // boundaries
  if (!(diag_loadmap_.boundaries_area_ && diag_loadmap_.boundaries_map_
        && diag_loadmap_.boundaries_district_)) {
    overall_level
      = std::max(overall_level, static_cast<int>(DiagnosticStatus::ERROR));
    parts.emplace_back("Corrupted boundary file.");
  } else {
    parts.emplace_back("Had boundary file");
  }

  // address
  if (!diag_loadmap_.address_.empty()) {
    parts.emplace_back(std::string("Address: ") + diag_loadmap_.address_);
  } else {
    overall_level
      = std::max(overall_level, static_cast<int>(DiagnosticStatus::ERROR));
    parts.emplace_back("No address");
  }

  // map_current list
  if (!diag_loadmap_.map_current_.empty()) {
    std::ostringstream oss;
    oss << "Current maps: ";
    for (size_t i = 0; i < diag_loadmap_.map_current_.size(); ++i) {
      oss << diag_loadmap_.map_current_[i];
      if (i + 1 < diag_loadmap_.map_current_.size()) oss << ", ";
    }
    parts.emplace_back(oss.str());
  } else {
    overall_level
      = std::max(overall_level, static_cast<int>(DiagnosticStatus::ERROR));
    parts.emplace_back("No map used");
  }

  // join parts to one short messages
  std::ostringstream out;
  for (size_t i = 0; i < parts.size(); ++i) {
    if (i) out << " - ";
    out << parts[i];
  }
  stat.summary(overall_level, out.str());

  stat.add("Point map count", diag_loadmap_.point_map_currents_);
  stat.add("Boundaries area", diag_loadmap_.boundaries_area_ ? "yes" : "no");
  stat.add("Boundaries map", diag_loadmap_.boundaries_map_ ? "yes" : "no");
  stat.add(
    "Boundaries district", diag_loadmap_.boundaries_district_ ? "yes" : "no");
  stat.add(
    "Address", diag_loadmap_.address_.empty() ? std::string("none")
                                              : diag_loadmap_.address_);
  stat.add(
    "Map current size", static_cast<int>(diag_loadmap_.map_current_.size()));
  for (size_t i = 0; i < diag_loadmap_.map_current_.size(); ++i) {
    stat.add(
      std::string("Current map ") + std::to_string(i + 1),
      diag_loadmap_.map_current_[i]);
  }
}

  /**
   *
   * CHECK & CREATE BOUNDARIES FILE
   *
   * **/
  bool
  AutoLoadMapSystem::checkBoundariesFile(const std::string & path_to_pcd)
{
  ROS_INFO("[AutoLoadMapSystem] Checking boundaries for all districts.");
  if(!fs::exists(path_to_pcd +"/district_boundaries.yaml")){
    diag_loadmap_.boundaries_district_ =false;
  }
  else{
    diag_loadmap_.boundaries_district_ = true;
  }
  for (const auto & district_entry :
       fs::directory_iterator(path_to_pcd))  // Hadong,hungyen
  {
    if (!fs::is_directory(district_entry)) continue;

    std::string district_path = district_entry.path().string();
    std::string district_name = district_entry.path().filename().string();

    ROS_INFO(
      "[checkBoundariesFile] Processing district: %s", district_name.c_str());

    for (auto & area_entry :
         fs::directory_iterator(district_path))  // Area_A,Area_B,..
    {
      if (!fs::is_directory(area_entry)) continue;
      std::string area_path     = area_entry.path().string();
      std::string yaml_map_path = area_path + "/map_boundaries.yaml";
      if (!fs::exists(yaml_map_path)) {
        diag_loadmap_.boundaries_map_=false;
        ROS_INFO(
          "[checkBoundariesFile] Generating map_boundaries.yaml for %s",
          area_path.c_str());
        generateYamlForMap(area_path);  //
      } else {
        diag_loadmap_.boundaries_map_ = true;
        ROS_INFO("[checkBoundariesFile] map_boundaries.yaml exists for %s",
          area_path.c_str());
      }
    }

    std::string yaml_area_path = district_path + "areas_boundaries.yaml";
    if (!fs::exists(yaml_area_path)) {
      ROS_INFO(
        "[checkBoundariesFile] Generating areas_boundaries.yaml for district "
        "%s",
        district_name.c_str());
      generateYamlForArea(district_path);
    } else {
        diag_loadmap_.boundaries_area_=true;
      ROS_INFO(
        "[checkBoundariesFile] areas_boundaries.yaml exists for %s",
        district_name.c_str());
    }
  }
  return true;
}

void AutoLoadMapSystem::generateYamlForMap(
  const std::string & area_path)  // Area_A,Area_B,..
{
  std::string yaml_path = area_path + "/map_boundaries.yaml";
  std::string pcd_path  = area_path + "/pcd";
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "files" << YAML::Value << YAML::BeginSeq;

  bool has_pcd = false;
  for (const auto & entry : fs::directory_iterator(pcd_path)) {
    if (entry.path().extension() != ".pcd") continue;
    has_pcd = true;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(entry.path().string(), *cloud) == -1) {
      ROS_ERROR(
        "[generateYamlForMap] Failed to load PCD file: %s",
        entry.path().string().c_str());
      continue;
    }

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    out << YAML::BeginMap;
    out << YAML::Key << "file" << YAML::Value
        << entry.path().filename().string();
    out << YAML::Key << "min" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << min_pt.x << min_pt.y << min_pt.z << YAML::EndSeq;
    out << YAML::Key << "max" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << max_pt.x << max_pt.y << max_pt.z << YAML::EndSeq;
    out << YAML::EndMap;
    ROS_INFO(
      "[generateYamlForMap] Added boundaries for %s",
      entry.path().filename().string().c_str());
  }

  if (!has_pcd) {
    diag_loadmap_.boundaries_map_ = false;
    ROS_ERROR(
      "[generateYamlForMap] No .pcd files found in %s", area_path.c_str());
  }

  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream yaml_file(yaml_path);
  if (yaml_file.is_open()) {
    yaml_file << out.c_str();
    yaml_file.close();
    diag_loadmap_.boundaries_map_ = true;
    ROS_INFO("[generateYamlForMap] Created %s", yaml_path.c_str());
  } else {
    diag_loadmap_.boundaries_map_ = false;
    ROS_ERROR("[generateYamlForMap] Failed to create %s", yaml_path.c_str());
  }
}

void AutoLoadMapSystem::generateYamlForArea(
  const std::string & district_path)  // hungyen,hadong
{
  std::string yaml_path = district_path + "/areas_boundaries.yaml";
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "district" << YAML::Value
      << fs::path(district_path).filename().string();
  out << YAML::Key << "areas" << YAML::Value << YAML::BeginSeq;

  bool has_areas = false;
  for (const auto & area_entry :
       fs::directory_iterator(district_path))  // Area_A,Area_B,..
  {
    if (!fs::is_directory(area_entry)) continue;
    std::string area_path = area_entry.path().string();
    std::string area_name = area_entry.path().filename().string();
    std::string map_yaml  = area_path + "/map_boundaries.yaml";

    if (!fs::exists(map_yaml)) {
      diag_loadmap_.boundaries_area_ = false;
      ROS_ERROR(
        "[generateYamlForArea] %s missing map_boundaries.yaml",
        area_name.c_str());
      continue;
    }

    has_areas         = true;
    YAML::Node config = YAML::LoadFile(map_yaml);

    Boundaries area_boundaries;
    area_boundaries.x_min = area_boundaries.y_min = area_boundaries.z_min
      = std::numeric_limits<float>::max();
    area_boundaries.x_max = area_boundaries.y_max = area_boundaries.z_max
      = std::numeric_limits<float>::lowest();

    if (config["files"]) {
      for (const auto & f : config["files"]) {
        auto min_node = f["min"];
        auto max_node = f["max"];
        if (!min_node || !max_node) continue;

        float min_x = min_node[0].as<float>();
        float min_y = min_node[1].as<float>();
        float min_z = min_node[2].as<float>();
        float max_x = max_node[0].as<float>();
        float max_y = max_node[1].as<float>();
        float max_z = max_node[2].as<float>();

        area_boundaries.x_min = std::min(area_boundaries.x_min, min_x);
        area_boundaries.y_min = std::min(area_boundaries.y_min, min_y);
        area_boundaries.z_min = std::min(area_boundaries.z_min, min_z);
        area_boundaries.x_max = std::max(area_boundaries.x_max, max_x);
        area_boundaries.y_max = std::max(area_boundaries.y_max, max_y);
        area_boundaries.z_max = std::max(area_boundaries.z_max, max_z);
      }
    }

    out << YAML::BeginMap;
    out << YAML::Key << "area" << YAML::Value << area_name;
    out << YAML::Key << "area_boundaries" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << area_boundaries.x_min << area_boundaries.y_min
        << area_boundaries.z_min << YAML::EndSeq;
    out << YAML::Key << "max" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << area_boundaries.x_max << area_boundaries.y_max
        << area_boundaries.z_max << YAML::EndSeq;
    out << YAML::EndMap;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream yaml_file(yaml_path);
  if (yaml_file.is_open()) {
    yaml_file << out.c_str();
    yaml_file.close();
    diag_loadmap_.boundaries_area_ = true;
    ROS_INFO("[generateYamlForArea] Created %s", yaml_path.c_str());
  } else {
    diag_loadmap_.boundaries_area_ = false;
    ROS_ERROR("[generateYamlForArea] Failed to create %s", yaml_path.c_str());
  }

  if (!has_areas) {
    ROS_ERROR(
      "[generateYamlForArea] No areas found in %s", district_path.c_str());
  }
}
/**
 *
 * SYSTEM ROS
 *
 * **/
void AutoLoadMapSystem::setupPublishers()
{
  point_cloud_pub_
    = nh_.advertise<sensor_msgs::PointCloud2>("/map/pointcloud_map", 1);
  filter_timer_ = nh_.createTimer(
    ros::Duration(0.05), &AutoLoadMapSystem::filterMapData, this);
}

void AutoLoadMapSystem::setupSubscribers()
{
  current_position_subscriber_ = nh_.subscribe(
    "current_position", 1, &AutoLoadMapSystem::poseCallback, this);
}

void AutoLoadMapSystem::poseCallback(
  const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  // if (!processing_gnss)
  // {
  float current_x = msg->pose.position.x;
  float current_y = msg->pose.position.y;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    position_queue_.push({current_x, current_y});
  }
  // }
  // processing_gnss = true;
  map_update_cv_.notify_one();
}

void AutoLoadMapSystem::startMapUpdateThread()
{
  map_update_thread_ = std::thread([this]() {
    while (!should_stop_) {
      std::pair<float, float> current_position;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        map_update_cv_.wait(
          lock, [this]() { return !position_queue_.empty() || should_stop_; });
        if (should_stop_) break;
        current_position = position_queue_.front();  // FIFO
        position_queue_.pop();
      }
      processMapUpdate(current_position.first, current_position.second);
    }
  });
}

/**
 *
 * EXTRACTING INFORMATION FROM A BOUNDARY YAML FILE
 * @brief detectCurrentArea
 * @brief findYamlConfig
 * @brief loadMapConfigurations
 * @brief loadAndUpdateArea
 * @brief isMapNeeded
 * @brief calculateDistanceToMap
 * @brief pointToLineDistance
 * @brief calculateDistance
 *
 * **/
DistrictInfo AutoLoadMapSystem::detectCurrentDistrict(float x, float y)
{
  DistrictInfo result;
  std::string boundaries_file = base_pcd_path_ + "/district_boundaries.yaml";

  if (!fs::exists(boundaries_file)) {
    ROS_ERROR("[detectCurrentDistrict] district_boundaries.yaml not found.");
    return result;
  }

  try {
    YAML::Node config = YAML::LoadFile(boundaries_file);
    if (!config["districts"]) {
      ROS_ERROR("[detectCurrentDistrict] No districts defined.");
      return result;
    }

    for (const auto & district : config["districts"]) {
      std::string district_name = district["name"].as<std::string>();
      auto min_xy               = district["min"].as<std::vector<float>>();
      auto max_xy               = district["max"].as<std::vector<float>>();

      if (
        x >= min_xy[0] && x <= max_xy[0] && y >= min_xy[1] && y <= max_xy[1]) {
        result.district_name = district_name;
        result.full_path     = base_pcd_path_ + "/" + district_name;
        result.min_xy        = {min_xy[0], min_xy[1]};
        result.max_xy        = {max_xy[0], max_xy[1]};
        // ROS_INFO("[detectCurrentDistrict] Detected district: %s",
        // district_name.c_str());
        return result;
      }
    }

    // ROS_WARN("[detectCurrentDistrict] Position (%.4f, %.4f) not in any
    // district.", x, y);
  } catch (const YAML::Exception & e) {
    ROS_ERROR("[detectCurrentDistrict] YAML error: %s", e.what());
  }

  return result;
}
AreaInfo AutoLoadMapSystem::detectCurrentArea(
  const DistrictInfo & district, float x, float y)
{
  AreaInfo result;
  if (district.district_name.empty()) return result;

  std::string areas_file = district.full_path + "/areas_boundaries.yaml";
  if (!fs::exists(areas_file)) {
    ROS_ERROR(
      "[detectCurrentArea] areas_boundaries.yaml not found in %s",
      district.full_path.c_str());
    return result;
  }

  try {
    YAML::Node config = YAML::LoadFile(areas_file);
    if (!config["areas"]) {
      ROS_ERROR("[detectCurrentArea] No areas defined.");
      return result;
    }

    for (const auto & area : config["areas"]) {
      std::string area_name = area["area"].as<std::string>();
      auto min = area["area_boundaries"]["min"].as<std::vector<float>>();
      auto max = area["area_boundaries"]["max"].as<std::vector<float>>();

      if (x >= min[0] && x <= max[0] && y >= min[1] && y <= max[1]) {
        result.area_name  = area_name;
        result.full_path  = district.full_path + "/" + area_name;
        result.boundaries = {min[0], min[1], min[2], max[0], max[1], max[2]};
        current_area_     = result.full_path;
        findYamlConfig(current_area_);
        ROS_INFO("[detectCurrentArea] Detected area: %s", area_name.c_str());
        return result;
      }
    }

    // ROS_WARN("[detectCurrentArea] Position (%.4f, %.4f) not in any area of
    // %s", x, y, district.district_name.c_str());
  } catch (const YAML::Exception & e) {
    ROS_ERROR("[detectCurrentArea] YAML error: %s", e.what());
  }

  return result;
}
MapInfo AutoLoadMapSystem::detectCurrentMap(
  const AreaInfo & area, float x, float y)
{
  MapInfo result;
  // std::cout <<"[detectCurrentMap]"<< area.area_name<<std::endl;
  if (area.area_name.empty()) return result;

  std::string maps_file = area.full_path + "/map_boundaries.yaml";
  if (!fs::exists(maps_file)) {
    ROS_ERROR(
      "[detectCurrentMap] map_boundaries.yaml not found in %s",
      area.full_path.c_str());
    return result;
  }

  try {
    YAML::Node config = YAML::LoadFile(maps_file);
    if (!config["files"]) {
      ROS_ERROR("[detectCurrentMap] No maps defined.");
      return result;
    }

    for (const auto & map_entry : config["files"]) {
      std::string map_name = map_entry["file"].as<std::string>();
      auto min             = map_entry["min"].as<std::vector<float>>();
      auto max             = map_entry["max"].as<std::vector<float>>();

      if (x >= min[0] && x <= max[0] && y >= min[1] && y <= max[1]) {
        result.map_name   = map_name;
        result.full_path  = area.full_path;
        result.boundaries = {min[0], min[1], min[2], max[0], max[1], max[2]};
        // current_area_ = area.full_path;
        result.cloud.reset(new pcl::PointCloud<PointT>);
        // findYamlConfig(current_area_);

        ROS_INFO(
          "[detectCurrentMap] Detected map: %s in %s ", map_name.c_str(),
          area.area_name.c_str());
        return result;
      }
    }

    // ROS_WARN("[detectCurrentMap] Position (%.4f, %.4f) not in any map of %s",
    // x, y, result.map_name.c_str());
  } catch (const YAML::Exception & e) {
    ROS_ERROR("[detectCurrentMap] YAML error: %s", e.what());
  }

  return result;
}

void AutoLoadMapSystem::findYamlConfig(const std::string & area_path)
{
  try {
    std::string boundaries_path = area_path + "/map_boundaries.yaml";
    // ROS_INFO("[findYamlConfig] boundaries_path: %s ",
    // boundaries_path.c_str()); Load and validate the YAML file
    YAML::Node config = YAML::LoadFile(boundaries_path);
    if (!config["files"]) {
      ROS_ERROR(
        "[findYamlConfig] Invalid YAML structure in %s",
        boundaries_path.c_str());
      return;
    }

    if (!loaded_boundaries_file_) { loadMapConfigurations(boundaries_path); }
  } catch (const std::exception & e) {
    ROS_ERROR("[findYamlConfig] Error: %s", e.what());
  }
}

void AutoLoadMapSystem::loadMapConfigurations(
  const std::string & map_config_path_)
{
  try {
    YAML::Node yaml_file = YAML::LoadFile(map_config_path_);
    for (const auto & file : yaml_file["files"]) {
      MapInfo map_info;
      map_info.map_name = file["file"].as<std::string>();
      auto min          = file["min"];
      auto max          = file["max"];

      map_info.boundaries.x_min = min[0].as<float>();
      map_info.boundaries.y_min = min[1].as<float>();
      map_info.boundaries.x_max = max[0].as<float>();
      map_info.boundaries.y_max = max[1].as<float>();
      map_info.isLoaded         = false;
      map_info.cloud.reset(new pcl::PointCloud<PointT>);

      available_maps_.push_back(map_info);
    }
    loaded_boundaries_file_ = true;
  } catch (const YAML::Exception & e) {
    ROS_ERROR(
      "[AutoLoadMapSystem] Error loading map configurations: %s", e.what());
  }
}

bool AutoLoadMapSystem::loadAndUpdateArea(const AreaInfo & area_info)
{
  if (area_info.area_name.empty()) {
    ROS_INFO(
      "[loadAndUpdateArea] Currently the car is not in %s area !!!! ",
      area_info.area_name.c_str());
    return false;
  }
  if (current_area_ == area_info.full_path) { return false; }

  std::string yaml_path = area_info.full_path + "/map_boundaries_area.yaml";
  if (yaml_path.empty()) { return false; }
  current_area_ = area_info.full_path;
  return true;
}

bool AutoLoadMapSystem::isVehicleInMap(float x, float y, const MapInfo & map)
{
  return (
    x >= map.boundaries.x_min && x <= map.boundaries.x_max
    && y >= map.boundaries.y_min && y <= map.boundaries.y_max);
}

bool AutoLoadMapSystem::isMapNeeded(
  float x, float y, const MapInfo & map, float & distance)
{
  distance = calculateDistanceToMap(x, y, map);
  return distance <= 50.0f;  // Load maps in [35, 50] meters range
}

float AutoLoadMapSystem::calculateDistanceToMap(
  float x, float y, const MapInfo & map)
float AutoLoadMapSystem::calculateDistanceToMap(
  float x, float y, const MapInfo & map)
{
  // check inside
  if (isVehicleInMap(x, y, map)) { return 0.0f; }

  float dist_to_left = pointToLineDistance(
    x, y, map.boundaries.x_min, map.boundaries.y_min, map.boundaries.x_min,
    map.boundaries.y_max);
  float dist_to_right = pointToLineDistance(
    x, y, map.boundaries.x_max, map.boundaries.y_min, map.boundaries.x_max,
    map.boundaries.y_max);
  float dist_to_bottom = pointToLineDistance(
    x, y, map.boundaries.x_min, map.boundaries.y_min, map.boundaries.x_max,
    map.boundaries.y_min);
  float dist_to_top = pointToLineDistance(
    x, y, map.boundaries.x_min, map.boundaries.y_max, map.boundaries.x_max,
    map.boundaries.y_max);
  return std::min({dist_to_left, dist_to_right, dist_to_bottom, dist_to_top});
}

float calculateDistance(float x1, float x2, float y1, float y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}
/**
 * Calculate the distance from a point P(px, py) to line segment AB(x1,y1 →
 * x2,y2). vector: AP = (px - x1, py - y1) AB = (x2 - x1, y2 - y1)
 *  - Calculate the projection ratio (param) of AP onto AB:
 *        param = (AP · AB) / |AB|²
 *  - There are 3 cases:
 *      param < 0 → the closest point is A
 *      param > 1 → the closest point is B
 *      0 ≤ param ≤ 1 → the closest point H lies on AB
 *
 *  --> Finally: distance = |PH| :is the shortest distance to line segment AB
 *
 *           P(px,py)
 *             *
 *            /|
 *           / |
 *     A *-----------------* B
 *             H (projected point)
 */
float AutoLoadMapSystem::pointToLineDistance(
  float px, float py, float x1, float y1, float x2, float y2)
{
  float A = px - x1;  // x-component of vector AP (v1)
  float B = py - y1;  // y-component of vector AP (v1)
  float C = x2 - x1;  // x-component of vector AB (v2)
  float D = y2 - y1;  // y-component of vector AB (v2)

  float dot    = A * C + B * D;  // profitlessg (xv1.xv2+yv1.yv2)
  float len_sq = C * C + D * D;  // |AB|²

  // Find projection points
  float param
    = -1;  // ratio of determining the position of point H on line segment AB
  if (len_sq != 0) param = dot / len_sq;

  float xx, yy;

  if (param < 0) {
    xx = x1;
    yy = y1;
  } else if (param > 1) {
    xx = x2;
    yy = y2;
  } else {  // The closest point lies on the line segment.
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  return calculateDistance(px, xx, py, yy);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr AutoLoadMapSystem::filterWithVoxelGrid(
  pcl::PointCloud<PointT>::Ptr cloud)
{
  if (!cloud || cloud->empty()) {
    ROS_ERROR("[MapAutoLoad] Input cloud is null or empty for voxel filtering");
    return pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  }

  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.25f, 0.25f, 0.25f);
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  voxel_filter.filter(*filtered_cloud);

  if (filtered_cloud->empty()) {
    ROS_ERROR(
      "[MapAutoLoad] VoxelGrid filter produced an empty cloud. Returning "
      "original...");
    return cloud;
  }
  return filtered_cloud;
}

/**
 *
 * UPDATE MAP
 * @brief processMapUpdate
 * @brief loadMapAsync
 * @brief unloadMapAsync
 * @brief getGroundHeight
 * @brief notifyPointsUpdate
 * @brief filterMapData
 *
 * **/
void AutoLoadMapSystem::processMapUpdate(float current_x, float current_y)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  DistrictInfo current_district = detectCurrentDistrict(current_x, current_y);
  if (current_district.district_name.empty()) {
    diag_loadmap_.address_="Address not found";
    ROS_WARN("[processMapUpdate] No district detected.");
    return;
  }

  AreaInfo current_area
    = detectCurrentArea(current_district, current_x, current_y);
  if (current_area.area_name.empty()) {
    diag_loadmap_.address_ = "No area detected";
    ROS_WARN(
      "[processMapUpdate] No area detected in %s.",
      current_district.district_name.c_str());
    return;
  }

  MapInfo current_map = detectCurrentMap(current_area, current_x, current_y);
  if (current_map.map_name.empty()) {
    diag_loadmap_.address_ = "No map detected";
    ROS_WARN(
      "[processMapUpdate] No map detected in %s.",
      current_area.area_name.c_str());
    return;
  }
  diag_loadmap_.address_ = current_district.district_name + "/"
                           + current_area.area_name + "/"
                           + current_map.map_name; 
    if (loadAndUpdateArea(current_area))
  {
    ROS_INFO(
      "[processMapUpdate] Switched to area: %s in district: %s",
      current_area.area_name.c_str(), current_district.district_name.c_str());
  }

  std::vector<std::string> maps_to_load, maps_to_unload, maps_to_publish;
  bool needs_update = false;
  std::string current_map_name;
  {
    std::lock_guard<std::mutex> lock(map_state_mutex_);
    MapInfo current_map = detectCurrentMap(current_area, current_x, current_y);
    if (!current_map.map_name.empty()) {
      current_map_name = current_map.map_name;
    }
    for (auto & map : available_maps_) {
      float distance;
      bool is_current  = isVehicleInMap(current_x, current_y, map);
      bool should_load = isMapNeeded(current_x, current_y, map, distance);
      bool already_loaded
        = (loaded_clouds_.find(map.map_name) != loaded_clouds_.end());

      // ROS_INFO("[processMapUpdate] Map %s: is_current=%d, should_load=%d,
      // already_loaded=%d, isLoaded=%d, isPublish=%d, distance=%.4f",
      //          map.map_name.c_str(), is_current, should_load,
      //          already_loaded, map.isLoaded, map.isPublish, distance);
    //   std::cout << "distance: " << distance << std::endl;
       
      if (!map.isLoaded)
      {
        if (should_load && !already_loaded) {
          // Force load current map if not already loaded
          maps_to_load.push_back(map.map_name);
          map.isLoaded  = true;
          map.isPublish = false;
          needs_update  = true;
          ROS_INFO(
            "[processMapUpdate] Force queue load %s (current map, dist: "
            "%.4f m)",
            map.map_name.c_str(), distance);
        }
      }

      if (map.isLoaded) {
        if (distance > 60.0f && !is_current) {
          maps_to_unload.push_back(map.map_name);
          maps_to_load.erase(
            std::remove(maps_to_load.begin(), maps_to_load.end(), map.map_name),
            maps_to_load.end());
          maps_to_publish.erase(
            std::remove(
              maps_to_publish.begin(), maps_to_publish.end(), map.map_name),
            maps_to_publish.end());
          map.isLoaded  = false;
          map.isPublish = false;
          needs_update  = true;
          ROS_INFO(
            "[processMapUpdate] Queue unload %s (dist: %.4f m)",
            map.map_name.c_str(), distance);
        } else if ((is_current || distance <= 40.0f) && !map.isPublish) {
          maps_to_publish.push_back(map.map_name);
          // map.isPublish = true;
          needs_update = true;
          ROS_INFO("[processMapUpdate] Queue publish %s", map.map_name.c_str());
        }
      }
    }
  }

  // ROS_INFO("[processMapUpdate] needs_update: %d, to_load: %zu, to_unload:
  // %zu, to_publish: %zu",
  //          needs_update, maps_to_load.size(), maps_to_unload.size(),
  //          maps_to_publish.size());

  if (needs_update) {
    std::vector<std::thread> threads;
    for (const auto & filename : maps_to_load) {
      threads.emplace_back([this, filename]() { loadMapAsync(filename); });
    }
    for (const auto & filename : maps_to_unload) {
      threads.emplace_back([this, filename]() { unloadMapAsync(filename); });
    }
    for (auto & t : threads) {
      if (t.joinable()) t.join();
    }
    updateMergedCloud(maps_to_publish);
  } else {
    // ROS_WARN("[processMapUpdate] No update needed, checking if current map
    // %s is published", current_map.map_name.c_str());
    std::lock_guard<std::mutex> lock(map_state_mutex_);
    for (auto & map : available_maps_) {
      if (map.map_name == current_map_name && map.isLoaded && !map.isPublish) {
        maps_to_publish.push_back(map.map_name);
        map.isPublish = true;
        updateMergedCloud(maps_to_publish);
        ROS_INFO(
          "[processMapUpdate] Forced publish of current map %s",
          map.map_name.c_str());
        break;
      }
    }
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> total_duration = end_total - start_total;
  ROS_INFO("[processMapUpdate] Total time: %.4f s", total_duration.count());
}
void AutoLoadMapSystem::loadMapAsync(const std::string & filename)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  auto start_time = std::chrono::high_resolution_clock::now();

  if (
    pcl::io::loadPCDFile<PointT>(current_area_ + "/pcd/" + filename, *cloud)
    == -1) {
    ROS_ERROR(
      "[AutoLoadMapSystem] Failed to load PCD file: %s", filename.c_str());
    return;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> load_dur = end_time - start_time;
  ROS_INFO(
    "[loadMapAsync] Loaded %s in %.4f s (%zu points)", filename.c_str(),
    load_dur.count(), cloud->size());

  {
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    if (loaded_clouds_.find(filename) != loaded_clouds_.end()) return;

    auto filtered = cloud;
    if (filtered && !filtered->empty()) {
      loaded_clouds_[filename] = filtered;
    } else {
      ROS_ERROR(
        "[loadMapAsync] Filtered cloud is empty for %s", filename.c_str());
    }
  }
}

void AutoLoadMapSystem::unloadMapAsync(const std::string & filename)
{
  {
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    auto it = loaded_clouds_.find(filename);
    if (it == loaded_clouds_.end()) {
      ROS_ERROR(
        "[unloadMapAsync] Map %s not found in loaded_clouds_",
        filename.c_str());
      return;
    }

    if (it->second && !it->second->empty()) {
      total_points_removed += it->second->size();
      if (removed_points_cloud) { *removed_points_cloud += *(it->second); }
    }
    loaded_clouds_.erase(it);
  }
}

void AutoLoadMapSystem::updateMergedCloud(
  const std::vector<std::string> & maps_to_publish)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  std::unique_lock<std::mutex> merge_lock(merge_mutex_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr new_merged_cloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  size_t total_points_added = 0;

  std::unordered_map<std::string, pcl::PointCloud<PointT>::Ptr> cloud_copies;
  {
    std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
    for (const auto & pair : loaded_clouds_) {
      if (pair.second && !pair.second->empty()) {
        cloud_copies[pair.first] = pcl::PointCloud<PointT>::Ptr(
          new pcl::PointCloud<PointT>(*pair.second));
      }
    }
  }

  // Merge all loaded maps
  std::unordered_set<std::string> active_maps;
  {
    std::lock_guard<std::mutex> state_lock(map_state_mutex_);
    for (const auto & map : available_maps_) {
      if (
        map.isLoaded && cloud_copies.find(map.map_name) != cloud_copies.end()) {
        active_maps.insert(map.map_name);
        diag_loadmap_.map_current_.push_back(map.map_name);
      }
    }
    
  }

  for (const auto & filename : active_maps) {
    auto it = cloud_copies.find(filename);
    if (it != cloud_copies.end() && it->second && !it->second->empty()) {
      *new_merged_cloud += *(it->second);
    }
  }

  // Process maps_to_publish
  {
    std::lock_guard<std::mutex> state_lock(map_state_mutex_);
    for (const auto & filename : maps_to_publish) {
      auto it = cloud_copies.find(filename);
      if (it != cloud_copies.end() && it->second && !it->second->empty()) {
        bool is_new_publish = false;
        for (auto & map : available_maps_) {
          if (map.map_name == filename && !map.isPublish && map.isLoaded) {
            map.isPublish  = true;
            is_new_publish = true;
            break;
          }
        }
        if (is_new_publish) {
          total_points_added += it->second->size();
          if (added_points_cloud) { *added_points_cloud += *(it->second); }
          ROS_INFO(
            "[updateMergedCloud] Added %zu points from map %s for publishing",
            it->second->size(), filename.c_str());
        }
      } else {
        ROS_ERROR(
          "[updateMergedCloud] Map %s in maps_to_publish but not in "
          "loaded_clouds_",
          filename.c_str());
      }
    }
  }

  if (!removed_points_cloud) {
    removed_points_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    ROS_INFO("[updateMergedCloud] Initialized removed_points_cloud");
  }

  // Thread-safe update of current_merged_cloud_
  {
    std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
    current_merged_cloud_ = new_merged_cloud;
  }

  // Publish merged cloud
  if (!new_merged_cloud->empty() || total_points_removed > 0) {
    sensor_msgs::PointCloud2 cloud_msg;
    try {
      pcl::toROSMsg(*new_merged_cloud, cloud_msg);
      cloud_msg.header.frame_id = "map";
      cloud_msg.header.stamp    = ros::Time::now();
      point_cloud_pub_.publish(cloud_msg);
      diag_loadmap_.point_map_currents_ = new_merged_cloud->size();
       ROS_INFO(
        "[updateMergedCloud] Published merged cloud with %zu points, removed "
        "%zu points",
        new_merged_cloud->size(), total_points_removed);
    } catch (const std::exception & e) {
      ROS_ERROR("[updateMergedCloud] Error publishing cloud: %s", e.what());
      return;
    }

    total_points_removed = 0;
  } else {
    sensor_msgs::PointCloud2 empty_msg;
    empty_msg.header.frame_id = "map";
    empty_msg.header.stamp    = ros::Time::now();
    point_cloud_pub_.publish(empty_msg);
    ROS_ERROR(
      "[updateMergedCloud] Published empty cloud (no valid maps to publish, no "
      "points removed)");
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end_time - start_time;
  ROS_INFO(
    "[updateMergedCloud] Cloud update completed in %.4f seconds",
    duration.count());
}

void AutoLoadMapSystem::filterMapData(const ros::TimerEvent &)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!current_merged_cloud_ || current_merged_cloud_->empty()) { return; }

    try {
      cloud_copy.reset(
        new pcl::PointCloud<pcl::PointXYZI>(*current_merged_cloud_));
    } catch (const std::exception & e) {
      ROS_ERROR("[filterMapData] Error creating cloud copy: %s", e.what());
      return;
    }
  }

  if (!cloud_copy || cloud_copy->empty()) { return; }

  has_map = true;

  try {
    // sensor_msgs::PointCloud2 cloud_msg;
    // pcl::toROSMsg(*cloud_copy, cloud_msg);
    // cloud_msg.header.frame_id = "map";
    // cloud_msg.header.stamp = ros::Time::now();
    // point_cloud_pub_.publish(cloud_msg);
  } catch (const std::exception & e) {
    ROS_ERROR("[filterMapData] Error publishing cloud: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "map_auto_load");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  AutoLoadMapSystem map_auto_load(nh, private_nh);
  ROS_INFO("[AutoLoadMapSystem] Ready to load map automatically.");

  ros::spin();
  return 0;
}