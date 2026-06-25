cmake_minimum_required(VERSION 3.8)
project(wall_follower_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(wall_follower_node
  src/wall_follower_node.cpp
)

ament_target_dependencies(wall_follower_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_msgs
)

install(TARGETS wall_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()