# Obstacle Distance Calculation Module (distance_calc)

A standalone, APF-independent C++ module that computes the **shortest distance**
to obstacles. **No external library dependencies** (standard C++17 only). It can
be used from any APF implementation.

> Scope: This module covers "environment → shortest distance" only.
> Repulsive/attractive forces and the APF itself are not included (handled separately).

---

## Features

### Inputs (two supported)
1. **Occupancy grid** — a 0/1 2D array built from a costmap or scan
2. **LiDAR point cloud** — a list of (x, y) [m] points centered on the robot
   (internally converted to a grid, then fed to the same distance transform)

### Outputs (four forms; use only what you need)
| Method | Returns | Use case |
|---|---|---|
| `distanceAt(r, c)` | `double` shortest distance [m] | Simplest. When you only need rho |
| `gradientAt(r, c)` | `Vec2` unit vector | "Escape direction" dphi/dx. Maps directly to the APF repulsive direction |
| `distanceField()` | `vector<double>` | Distance map over the whole grid |
| `distanceAtWorld(x, y, ox, oy)` | `double` | Distance at an arbitrary world coordinate |

Coordinate convention: **x = column direction, y = row direction**.
`gradientAt` points away from obstacles.

---

## Usage (minimal)

```cpp
#include "distance_calc/distance_calc.hpp"
using namespace distance_calc;

DistanceCalculator calc(0.5);          // resolution [m/cell]

// --- When passing an occupancy grid ---
calc.setFromOccupancyGrid(grid);

// --- When passing a LiDAR point cloud ---
GridSpec spec;                          // resolution, half_size_m
calc.setFromPointCloud(points, spec);

// Robot position (the grid center for point-cloud input)
std::size_t r = calc.centerRow();
std::size_t c = calc.centerCol();

double rho = calc.distanceAt(r, c);     // shortest distance
Vec2   dir = calc.gradientAt(r, c);     // escape direction
```

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

To use it from a teammate's APF package:
```cmake
target_link_libraries(<their_target> distance_calc)
```

---

## How it works (distance transform)

The module applies a **two-pass distance transform (Felzenszwalb's algorithm)**
to the occupancy grid, computing in one pass how many meters each cell is from
the nearest obstacle. This is faster than brute force, and it yields both the
distance map over the whole grid and the gradient (escape direction) at once.

> Implementation note: Using infinity as the initial cost produces NaN
> internally, so a finite value larger than the board size is used instead.

## Limitations
- Distances are quantized to the grid resolution (use a smaller resolution if
  higher precision is needed).
- `gradientAt` returns {0, 0} when the gradient cancels out, e.g. when an
  obstacle occupies only a single cell (typically not an issue with real LiDAR,
  where obstacles span multiple cells).
