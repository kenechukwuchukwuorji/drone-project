// distance_calc.cpp
#include "distance_calc/distance_calc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace distance_calc {

namespace {

// 1次元の二乗距離変換 (Felzenszwalb & Huttenlocher)。
// f: 入力コスト列, d: 出力(各点の最近接の二乗距離)。
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

  // INFの代わりに「盤面より十分大きい有限値」を使う(dt1dでのinf-inf=NaN回避)。
  const double BIG =
      static_cast<double>(rows_) * rows_ + static_cast<double>(cols_) * cols_ + 1.0;

  std::vector<double> f(total, 0.0);
  for (std::size_t i = 0; i < total; ++i) {
    f[i] = (grid.data[i] != 0) ? 0.0 : BIG;
  }

  // 列方向 → 行方向 の2パス
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

  // 二乗距離(セル単位) -> 距離[m]
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
  const int n = 2 * half + 1;  // 奇数 → 中心セルが存在
  resolution_ = spec.resolution;

  OccupancyGrid grid;
  grid.rows = static_cast<std::size_t>(n);
  grid.cols = static_cast<std::size_t>(n);
  grid.data.assign(static_cast<std::size_t>(n) * n, 0);

  // ロボットはグリッド中心。点群の (x,y)[m] をセルに落とす。
  // 規約: x=col方向, y=row方向。
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

  // row方向の勾配(端は片側差分、内部は中央差分)
  double gr;
  if (r == 0) {
    gr = val(r + 1, c) - val(r, c);
  } else if (r == rows_ - 1) {
    gr = val(r, c) - val(r - 1, c);
  } else {
    gr = 0.5 * (val(r + 1, c) - val(r - 1, c));
  }
  // col方向の勾配
  double gc;
  if (c == 0) {
    gc = val(r, c + 1) - val(r, c);
  } else if (c == cols_ - 1) {
    gc = val(r, c) - val(r, c - 1);
  } else {
    gc = 0.5 * (val(r, c + 1) - val(r, c - 1));
  }

  const double norm = std::hypot(gr, gc);
  if (norm < 1e-9) return Vec2{0.0, 0.0};  // 局所min等
  // 単位ベクトル: x=col方向, y=row方向
  return Vec2{gc / norm, gr / norm};
}

double DistanceCalculator::distanceAtWorld(double wx, double wy,
                                           double origin_x,
                                           double origin_y) const {
  if (rows_ == 0 || cols_ == 0) return 0.0;
  // ワールド -> セル。origin は左下セル中心のワールド座標とみなす。
  int c = static_cast<int>(std::lround((wx - origin_x) / resolution_));
  int r = static_cast<int>(std::lround((wy - origin_y) / resolution_));
  // グリッド外は縁にクランプ
  c = std::clamp(c, 0, static_cast<int>(cols_) - 1);
  r = std::clamp(r, 0, static_cast<int>(rows_) - 1);
  return dist_field_[static_cast<std::size_t>(r) * cols_ + c];
}

}  // namespace distance_calc
