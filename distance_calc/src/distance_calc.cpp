// distance_calc.cpp
#include "distance_calc/distance_calc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace distance_calc {

namespace {

// 1D squared distance transform (Felzenszwalb & Huttenlocher).
// f: input cost array, d: output (squared distance to the nearest point at each index).
void dt1d(const std::vector<double>& f, std::vector<double>& d) {
  const std::size_t n = f.size();
  d.assign(n, 0.0);
  if (n == 0) return;

  std::vector<std::size_t> v(n, 0);
  std::vector<double> z(n + 1, 0.0);
  std::size_t k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();

  for (std::size_t q = 1; q < n; ++q) {
    double s;
    while (true) {
      const std::size_t vk = v[k];
      s = ((f[q] + static_cast<double>(q) * q) -
           (f[vk] + static_cast<double>(vk) * vk)) /
          (2.0 * static_cast<double>(q) - 2.0 * static_cast<double>(vk));
      if (s <= z[k]) {
        if (k == 0) break;
        --k;
      } else {
        break;
      }
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }

  k = 0;
  for (std::size_t q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<double>(q)) ++k;
    const double diff = static_cast<double>(q) - static_cast<double>(v[k]);
    d[q] = diff * diff + f[v[k]];
  }
}

}  // namespace

DistanceCalculator::DistanceCalculator(double resolution)
    : resolution_(resolution) {}

void DistanceCalculator::buildDistanceField(const OccupancyGrid& grid) {
  rows_ = grid.rows;
  cols_ = grid.cols;
  const std::size_t total = rows_ * cols_;

  // Use a "large finite value larger than the board" instead of INF
  // (avoids inf - inf = NaN inside dt1d).
  const double BIG =
      static_cast<double>(rows_) * rows_ + static_cast<double>(cols_) * cols_ + 1.0;

  std::vector<double> f(total, 0.0);
  for (std::size_t i = 0; i < total; ++i) {
    f[i] = (grid.data[i] != 0) ? 0.0 : BIG;
  }

  // Two passes: along columns, then along rows.
  std::vector<double> col_in(rows_), col_out(rows_);
  for (std::size_t c = 0; c < cols_; ++c) {
    for (std::size_t r = 0; r < rows_; ++r) col_in[r] = f[r * cols_ + c];
    dt1d(col_in, col_out);
    for (std::size_t r = 0; r < rows_; ++r) f[r * cols_ + c] = col_out[r];
  }
  std::vector<double> row_in(cols_), row_out(cols_);
  for (std::size_t r = 0; r < rows_; ++r) {
    for (std::size_t c = 0; c < cols_; ++c) row_in[c] = f[r * cols_ + c];
    dt1d(row_in, row_out);
    for (std::size_t c = 0; c < cols_; ++c) f[r * cols_ + c] = row_out[c];
  }

  // Squared distance (in cells) -> distance [m].
  dist_field_.assign(total, 0.0);
  for (std::size_t i = 0; i < total; ++i) {
    dist_field_[i] = std::sqrt(f[i]) * resolution_;
  }
}

void DistanceCalculator::setFromOccupancyGrid(const OccupancyGrid& grid) {
  buildDistanceField(grid);
  center_r_ = rows_ / 2;
  center_c_ = cols_ / 2;
}

void DistanceCalculator::setFromPointCloud(
    const std::vector<std::pair<double, double>>& points, const GridSpec& spec) {
  const int half = static_cast<int>(spec.half_size_m / spec.resolution);
  const int n = 2 * half + 1;  // odd -> a center cell exists
  resolution_ = spec.resolution;

  OccupancyGrid grid;
  grid.rows = static_cast<std::size_t>(n);
  grid.cols = static_cast<std::size_t>(n);
  grid.data.assign(static_cast<std::size_t>(n) * n, 0);

  // The robot is at the grid center. Drop each point (x,y)[m] into a cell.
  // Convention: x = col direction, y = row direction.
  for (const auto& p : points) {
    const double px = p.first;   // x
    const double py = p.second;  // y
    const int dc = static_cast<int>(std::lround(px / spec.resolution));
    const int dr = static_cast<int>(std::lround(py / spec.resolution));
    const int r = half + dr;
    const int c = half + dc;
    if (r >= 0 && r < n && c >= 0 && c < n) {
      grid.data[static_cast<std::size_t>(r) * n + c] = 1;
    }
  }

  buildDistanceField(grid);
  center_r_ = static_cast<std::size_t>(half);
  center_c_ = static_cast<std::size_t>(half);
}

double DistanceCalculator::distanceAt(std::size_t r, std::size_t c) const {
  if (r >= rows_ || c >= cols_) return 0.0;
  return dist_field_[r * cols_ + c];
}

Vec2 DistanceCalculator::gradientAt(std::size_t r, std::size_t c) const {
  if (rows_ == 0 || cols_ == 0) return Vec2{0.0, 0.0};

  auto val = [&](std::size_t rr, std::size_t cc) {
    return dist_field_[rr * cols_ + cc];
  };

  // Gradient along the row direction (one-sided at edges, central inside).
  double gr;
  if (r == 0) {
    gr = val(r + 1, c) - val(r, c);
  } else if (r == rows_ - 1) {
    gr = val(r, c) - val(r - 1, c);
  } else {
    gr = 0.5 * (val(r + 1, c) - val(r - 1, c));
  }
  // Gradient along the column direction.
  double gc;
  if (c == 0) {
    gc = val(r, c + 1) - val(r, c);
  } else if (c == cols_ - 1) {
    gc = val(r, c) - val(r, c - 1);
  } else {
    gc = 0.5 * (val(r, c + 1) - val(r, c - 1));
  }

  const double norm = std::hypot(gr, gc);
  if (norm < 1e-9) return Vec2{0.0, 0.0};  // local minimum, etc.
  // Unit vector: x = col direction, y = row direction.
  return Vec2{gc / norm, gr / norm};
}

double DistanceCalculator::distanceAtWorld(double wx, double wy,
                                           double origin_x,
                                           double origin_y) const {
  if (rows_ == 0 || cols_ == 0) return 0.0;
  // World -> cell. origin is taken as the world coordinate of the
  // bottom-left cell center.
  int c = static_cast<int>(std::lround((wx - origin_x) / resolution_));
  int r = static_cast<int>(std::lround((wy - origin_y) / resolution_));
  // Clamp out-of-grid queries to the border.
  c = std::clamp(c, 0, static_cast<int>(cols_) - 1);
  r = std::clamp(r, 0, static_cast<int>(rows_) - 1);
  return dist_field_[static_cast<std::size_t>(r) * cols_ + c];
}

}  // namespace distance_calc
