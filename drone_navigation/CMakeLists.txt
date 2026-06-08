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
find_package (geometry_msgs REQUIRED)
find_package (Eigen3 REQUIRED)

add_executable(apf src/apf.cpp)
ament_target_dependencies(apf rclcpp nav_msgs geometry_msgs)

target_link_libraries(apf Eigen3::Eigen)

install(PROGRAMS 

  script/map.py

  DESTINATION lib/${PROJECT_NAME}

)

install(TARGETS
  apf
  DESTINATION lib/${PROJECT_NAME}

)

ament_package()
