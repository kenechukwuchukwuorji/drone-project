// distance_calc.hpp
// ============================================================================
//  最短距離計算モジュール (障害物までの距離)  — APF非依存・単体で使える
// ----------------------------------------------------------------------------
//  役割: 「環境(占有グリッド or LiDAR点群)」を受け取り、
//        「障害物までの最短距離」に関する情報を返すだけ。
//        APF(斥力・引力)は一切含まない。どんなAPF実装からでも利用可能。
//
//  入力 (2通りに対応):
//    1. 占有グリッド (costmapやscanから作った 0/1 の2D配列)
//    2. LiDAR点群    (ロボット中心の (x,y) [m] のリスト)
//       → 内部でグリッドに変換してから距離変換に流す(処理は一本化)
//
//  出力 (友達のAPFが何を使ってもいいよう4通り提供):
//    A. distanceAt(r,c)      … 指定セルの最短距離 [m] (数値ひとつ)
//    B. gradientAt(r,c)      … 「障害物から逃げる向き」= ∂ρ/∂x の単位ベクトル
//    C. distanceField()      … グリッド全体の距離マップ [m] (2D配列)
//    D. distanceAtWorld(x,y) … 任意のワールド座標での距離 [m] (近傍補間)
//
//  外部ライブラリ依存なし(標準C++17のみ)。距離変換は自前実装。
// ============================================================================

#ifndef DISTANCE_CALC_DISTANCE_CALC_HPP
#define DISTANCE_CALC_DISTANCE_CALC_HPP

#include <vector>
#include <cstddef>
#include <utility>

namespace distance_calc {

// 2次元ベクトル(向きなどに使用)
struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

// 占有グリッド(row-major の1次元配列で保持)
//   value == 0 : 空き / value != 0 : 障害物
struct OccupancyGrid {
  std::vector<int> data;   // サイズ = rows * cols
  std::size_t rows = 0;
  std::size_t cols = 0;

  int at(std::size_t r, std::size_t c) const { return data[r * cols + c]; }
};

// LiDAR等の点群を保持するためのグリッド設定。
// ロボットを中心に、一辺 (2*half_size_m) の正方グリッドを作る。
struct GridSpec {
  double resolution = 0.05;   // [m/cell]
  double half_size_m = 2.0;   // ロボット中心から端までの距離 [m]
};

class DistanceCalculator {
 public:
  // resolution: 1セルの大きさ [m/cell]。距離をメートルで返すために使う。
  explicit DistanceCalculator(double resolution);

  // ---- 入力1: 占有グリッドから距離場を構築 ----
  void setFromOccupancyGrid(const OccupancyGrid& grid);

  // ---- 入力2: LiDAR点群から距離場を構築 ----
  // points: ロボット中心の (x,y) [m]。spec で範囲・解像度を指定。
  // ロボットはグリッド中心に置かれる。
  void setFromPointCloud(const std::vector<std::pair<double, double>>& points,
                         const GridSpec& spec);

  // 中心(ロボット位置)のセル座標を返す。点群入力時に有用。
  std::size_t centerRow() const { return center_r_; }
  std::size_t centerCol() const { return center_c_; }

  bool ready() const { return rows_ > 0 && cols_ > 0; }
  std::size_t rows() const { return rows_; }
  std::size_t cols() const { return cols_; }

  // ---- 出力A: 指定セルの最短距離 [m] ----
  double distanceAt(std::size_t r, std::size_t c) const;

  // ---- 出力B: 逃げる向き ∂ρ/∂x(単位ベクトル, x=col方向, y=row方向) ----
  // 障害物から離れる方向。距離マップの勾配を正規化したもの。
  // 勾配がほぼ0(局所min等)のときは {0,0} を返す。
  Vec2 gradientAt(std::size_t r, std::size_t c) const;

  // ---- 出力C: グリッド全体の距離マップ [m] (row-major) ----
  const std::vector<double>& distanceField() const { return dist_field_; }

  // ---- 出力D: 任意ワールド座標での距離 [m] ----
  // origin はグリッド原点[m](左下セル中心のワールド座標)。
  // グリッド外を指定した場合は最も近い縁の値を返す。
  double distanceAtWorld(double wx, double wy, double origin_x,
                         double origin_y) const;

 private:
  void buildDistanceField(const OccupancyGrid& grid);

  double resolution_;
  std::size_t rows_ = 0;
  std::size_t cols_ = 0;
  std::size_t center_r_ = 0;
  std::size_t center_c_ = 0;
  std::vector<double> dist_field_;  // 各セルの最短距離 [m]
};

}  // namespace distance_calc

#endif  // DISTANCE_CALC_DISTANCE_CALC_HPP
