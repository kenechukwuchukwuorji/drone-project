cmake_minimum_required(VERSION 3.10)
project(distance_calc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Shortest-distance calculation library ---
add_library(distance_calc src/distance_calc.cpp)
target_include_directories(distance_calc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
