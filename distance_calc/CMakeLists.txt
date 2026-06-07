cmake_minimum_required(VERSION 3.10)
project(distance_calc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 最短距離計算ライブラリ(外部依存なし) ---
add_library(distance_calc src/distance_calc.cpp)
target_include_directories(distance_calc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ============================================================
# 友達のAPFパッケージから使う場合:
#   target_link_libraries(<友達のターゲット> distance_calc)
#   #include "distance_calc/distance_calc.hpp"
# ROS非依存なので、ROS2パッケージにもそのまま同梱可能。
# ============================================================
