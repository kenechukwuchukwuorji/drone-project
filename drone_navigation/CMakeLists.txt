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
find_package(tf2_msgs REQUIRED)

add_executable(apf src/apf.cpp)
ament_target_dependencies(apf rclcpp nav_msgs geometry_msgs tf2_msgs)

add_executable(apf_improved src/apf_improved.cpp)
ament_target_dependencies(apf_improved rclcpp nav_msgs geometry_msgs tf2_msgs)

add_executable(apf_wall_following src/apf_wall_following.cpp)
ament_target_dependencies(apf_wall_following rclcpp nav_msgs geometry_msgs tf2_msgs)

target_link_libraries(apf Eigen3::Eigen)
target_link_libraries(apf_improved Eigen3::Eigen)
target_link_libraries(apf_wall_following Eigen3::Eigen)

install(PROGRAMS 

  script/map.py

  DESTINATION lib/${PROJECT_NAME}

)

install(TARGETS
  apf
  apf_improved
  apf_wall_following
  DESTINATION lib/${PROJECT_NAME}

)

install(DIRECTORY 
  launch
  
  DESTINATION share/${PROJECT_NAME}

)

ament_package()
