cmake_minimum_required(VERSION 2.8.3)
project(grid_map_demos_itu)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  cv_bridge
  octomap_msgs
  filters
  pcl_ros
  pcl_msgs
  pcl_conversions
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(octomap REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)



#add_executable(
#  octomap_to_gridmap_demo
#  src/octomap_to_gridmap_demo_node.cpp
#  src/OctomapToGridmapDemo.cpp
#)



#target_link_libraries(
#  octomap_to_gridmap_demo
#  ${catkin_LIBRARIES}
#  ${OCTOMAP_LIBRARIES}
#)


add_executable(pointcloud_to_gridmap_node src/heterojen_robot/pc2gridmap.cpp src/heterojen_robot/pc2gridmap.hpp src/heterojen_robot/pointcloud_to_gridmap_node.cpp)
target_link_libraries(pointcloud_to_gridmap_node  ${PCL_LIBRARIES}  ${catkin_LIBRARIES} )

#############
## Install ##
#############


# Mark executables and/or libraries for installation
install(
  TARGETS pointcloud_to_gridmap_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY config data doc launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
