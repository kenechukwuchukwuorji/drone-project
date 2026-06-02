cmake_minimum_required(VERSION 3.8)
project(drone_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (nav_msgs REQUIRED)
find_package (map_msgs REQUIRED)

install(PROGRAMS 

  script/map.py

  DESTINATION lib/${PROJECT_NAME}

)

ament_package()
