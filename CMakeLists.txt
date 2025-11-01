cmake_minimum_required(VERSION 3.0.2)
project(iturover_odometry_assignment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs tf tf2_ros tf2_geometry_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(odometry_fuser_node
  src/odometry_node.cpp
  src/kalman_filter.cpp
)

target_link_libraries(odometry_fuser_node
  ${catkin_LIBRARIES}
)

install(TARGETS odometry_fuser_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)