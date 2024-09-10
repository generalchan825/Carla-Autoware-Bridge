cmake_minimum_required(VERSION 3.5)
project(lidar2cam)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare the executable
add_executable(lidar2cam_node src/lidar2cam_node.cpp)

# Link against the libraries
ament_target_dependencies(lidar2cam_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  tf2_ros
  geometry_msgs
  Eigen3
  OpenCV
)

install(TARGETS
  lidar2cam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
