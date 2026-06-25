cmake_minimum_required(VERSION 3.8)
project(obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(obstacle_detector
  src/obstacle_detector.cpp
)

ament_target_dependencies(obstacle_detector
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
