cmake_minimum_required(VERSION 3.0.2)
project(auto_loadmap)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen 
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions

  # ndt_omp
)

find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED)
find_package(YAML-CPP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs  pcl_conversions tf2_eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS} 
  ${GeographicLib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(auto_load_map src/auto_update.cpp)
add_executable(preprocessing src/pre_processing.cpp)

add_dependencies(auto_load_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_load_map 
  ${catkin_LIBRARIES} 
  ${YAML_CPP_LIBRARIES} 
  ${GeographicLib_LIBRARIES} 
  ${PCL_LIBRARIES}
)

add_dependencies(preprocessing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(preprocessing
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(
  TARGETS auto_load_map preprocessing
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES include/auto_loadmap/auto_update.h include/auto_loadmap/pre_processing.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)