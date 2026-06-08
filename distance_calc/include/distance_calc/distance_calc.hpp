// distance_calc.hpp
// ============================================================================
//  Obstacle distance calculation module — APF-independent, usable standalone
// ----------------------------------------------------------------------------
//  Purpose: Takes an environment (occupancy grid or LiDAR point cloud) and
//           returns information about the shortest distance to obstacles.
//           Contains no APF logic (no repulsive/attractive forces). Can be
//           used from any APF implementation.
//
//  Inputs (two supported):
//    1. Occupancy grid (a 0/1 2D array built from a costmap or scan)
//    2. LiDAR point cloud (a list of (x, y) [m] points centered on the robot)
//       -> internally converted to a grid, then fed to the distance transform
//          (processing is unified)
//
//  Outputs (four forms, so any APF can use whatever it needs):
//    A. distanceAt(r, c)      ... shortest distance [m] at a cell (a scalar)
//    B. gradientAt(r, c)      ... "direction away from obstacles" = unit vector
//                                  of dphi/dx
//    C. distanceField()       ... distance map over the whole grid [m] (2D)
//    D. distanceAtWorld(x, y) ... distance [m] at any world coordinate
//
//  No external library dependencies (standard C++17 only). The distance
//  transform is implemented in-house.
// ============================================================================

#ifndef DISTANCE_CALC_DISTANCE_CALC_HPP
#define DISTANCE_CALC_DISTANCE_CALC_HPP

#include <vector>
#include <cstddef>
#include <utility>

namespace distance_calc {

// 2D vector (used for directions, etc.)
struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

// Occupancy grid (stored as a row-major 1D array)
//   value == 0 : free / value != 0 : obstacle
struct OccupancyGrid {
  std::vector<int> data;   // size = rows * cols
  std::size_t rows = 0;
  std::size_t cols = 0;

  int at(std::size_t r, std::size_t c) const { return data[r * cols + c]; }
};

// Grid settings for holding a point cloud (e.g. from LiDAR).
// Builds a square grid of side (2 * half_size_m) centered on the robot.
struct GridSpec {
  double resolution = 0.05;   // [m/cell]
  double half_size_m = 2.0;   // distance from robot center to edge [m]
};

class DistanceCalculator {
 public:
  // resolution: size of one cell [m/cell]. Used to return distances in meters.
  explicit DistanceCalculator(double resolution);

  // ---- Input 1: build the distance field from an occupancy grid ----
  void setFromOccupancyGrid(const OccupancyGrid& grid);

  // ---- Input 2: build the distance field from a LiDAR point cloud ----
  // points: (x, y) [m] centered on the robot. spec sets range and resolution.
  // The robot is placed at the grid center.
  void setFromPointCloud(const std::vector<std::pair<double, double>>& points,
                         const GridSpec& spec);

  // Return the center (robot position) cell coordinates. Useful for cloud input.
  std::size_t centerRow() const { return center_r_; }
  std::size_t centerCol() const { return center_c_; }

  bool ready() const { return rows_ > 0 && cols_ > 0; }
  std::size_t rows() const { return rows_; }
  std::size_t cols() const { return cols_; }

  // ---- Output A: shortest distance [m] at a given cell ----
  double distanceAt(std::size_t r, std::size_t c) const;

  // ---- Output B: escape direction dphi/dx (unit vector, x=col, y=row) ----
  // Direction away from obstacles. The normalized gradient of the distance map.
  // Returns {0, 0} when the gradient is nearly zero (e.g. a local minimum).
  Vec2 gradientAt(std::size_t r, std::size_t c) const;

  // ---- Output C: distance map over the whole grid [m] (row-major) ----
  const std::vector<double>& distanceField() const { return dist_field_; }

  // ---- Output D: distance [m] at an arbitrary world coordinate ----
  // origin is the grid origin [m] (world coordinate of the lower-left cell center).
  // If the point is outside the grid, the value of the nearest edge is returned.
  double distanceAtWorld(double wx, double wy, double origin_x,
                         double origin_y) const;

 private:
  void buildDistanceField(const OccupancyGrid& grid);

  double resolution_;
  std::size_t rows_ = 0;
  std::size_t cols_ = 0;
  std::size_t center_r_ = 0;
  std::size_t center_c_ = 0;
  std::vector<double> dist_field_;  // shortest distance [m] for each cell
};

}  // namespace distance_calc

#endif  // DISTANCE_CALC_DISTANCE_CALC_HPPNCE_CALC_HPP
