// Copyright (c) 2024 Pavan N K
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Benchmark: polygon rasterization approaches
//   - old: bounding-box + isPointInsidePolygon (O(width*height) per cell)
//   - new: scanline putFilled, initial version (per-scanline xs allocation)
//   - opt: scanline putFilled, optimized (hoisted xs, precomputed edges, std::fill spans)
//   - opencv: cv::fillPoly (reference, compiled in only when HAVE_OPENCV is defined)
//
// Build without OpenCV:
//   g++ -std=c++17 -O2 -o benchmark_rasterization benchmark_rasterization.cpp
//
// Build with OpenCV (for full comparison):
//   g++ -std=c++17 -O2 -DHAVE_OPENCV -o benchmark_rasterization \
//       benchmark_rasterization.cpp $(pkg-config --cflags --libs opencv4)
//
// Run:
//   ./benchmark_rasterization
//
// What is measured:
//   Each benchmark call includes grid reset (data.assign) + fill.
//   The reset is included in every variant to ensure fair comparison:
//   all implementations measure equivalent work (reset + fill).
//   Timing uses std::chrono::high_resolution_clock.
//   Results are reported as mean/min/max over ITER iterations.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#ifdef HAVE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

using Clock = std::chrono::high_resolution_clock;
using us = std::chrono::microseconds;

// ---------------------------------------------------------------------------
// Minimal occupancy-grid stand-in (no ROS)
// ---------------------------------------------------------------------------
struct GridInfo
{
  double origin_x{0.0}, origin_y{0.0}, resolution{1.0};
  unsigned int width{0}, height{0};
};

struct OccGrid
{
  GridInfo info;
  std::vector<int8_t> data;

  void reset(unsigned int w, unsigned int h, double res = 1.0)
  {
    info.width = w;
    info.height = h;
    info.resolution = res;
    info.origin_x = 0.0;
    info.origin_y = 0.0;
    data.assign(static_cast<std::size_t>(w) * h, 0);
  }
};

// ---------------------------------------------------------------------------
// Helper: is-point-inside polygon (ray-casting, matches upstream)
// ---------------------------------------------------------------------------
static bool isPointInsidePolygon(
  double px, double py,
  const std::vector<std::pair<double, double>> & pts)
{
  bool inside = false;
  const std::size_t n = pts.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    double xi = pts[i].first, yi = pts[i].second;
    double xj = pts[j].first, yj = pts[j].second;
    if (((yi > py) != (yj > py)) &&
      (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

// ---------------------------------------------------------------------------
// OLD approach: bounding-box + isPointInside
// ---------------------------------------------------------------------------
static void fillOld(
  OccGrid & grid,
  const std::vector<std::pair<double, double>> & pts,
  int8_t val)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (auto & p : pts) {
    min_x = std::min(min_x, p.first);
    min_y = std::min(min_y, p.second);
    max_x = std::max(max_x, p.first);
    max_y = std::max(max_y, p.second);
  }

  const auto & info = grid.info;
  const double res = info.resolution;
  const double ox = info.origin_x, oy = info.origin_y;

  auto toMapX = [&](double wx) {
      return static_cast<unsigned int>((wx - ox) / res);
    };
  auto toMapY = [&](double wy) {
      return static_cast<unsigned int>((wy - oy) / res);
    };

  unsigned int mx0 = (min_x >= ox) ? toMapX(min_x) : 0u;
  unsigned int my0 = (min_y >= oy) ? toMapY(min_y) : 0u;
  unsigned int mx1 = std::min(
    (max_x >= ox) ? toMapX(max_x) : info.width - 1,
    info.width - 1);
  unsigned int my1 = std::min(
    (max_y >= oy) ? toMapY(max_y) : info.height - 1,
    info.height - 1);

  for (unsigned int my = my0; my <= my1; my++) {
    for (unsigned int mx = mx0; mx <= mx1; mx++) {
      double wx = ox + (mx + 0.5) * res;
      double wy = oy + (my + 0.5) * res;
      if (isPointInsidePolygon(wx, wy, pts)) {
        grid.data[my * info.width + mx] = val;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// NEW approach (initial): scanline with per-scanline xs allocation
// (matches the pre-optimization putFilled in vector_object_shapes.cpp)
// ---------------------------------------------------------------------------
static void fillScanline(
  OccGrid & grid,
  const std::vector<std::pair<double, double>> & pts,
  int8_t val)
{
  const std::size_t n = pts.size();
  if (n < 3) {return;}

  const auto & info = grid.info;
  const double origin_x = info.origin_x;
  const double origin_y = info.origin_y;
  const double res = info.resolution;

  std::vector<double> vx(n), vy(n);
  for (std::size_t i = 0; i < n; i++) {
    vx[i] = (pts[i].first - origin_x) / res - 0.5;
    vy[i] = (pts[i].second - origin_y) / res - 0.5;
  }

  int y_min = static_cast<int>(std::ceil(*std::min_element(vy.begin(), vy.end())));
  int y_max = static_cast<int>(std::floor(*std::max_element(vy.begin(), vy.end())));
  y_min = std::max(y_min, 0);
  y_max = std::min(y_max, static_cast<int>(info.height) - 1);

  const int map_width = static_cast<int>(info.width);

  for (int y = y_min; y <= y_max; y++) {
    std::vector<double> xs;   // <-- allocated per scanline (the hot allocation)
    xs.reserve(n);

    for (std::size_t i = 0; i < n; i++) {
      std::size_t j = (i + 1) % n;
      double y0 = vy[i], y1 = vy[j];
      double x0 = vx[i], x1 = vx[j];

      if (y0 == y1) {continue;}
      if (y < std::min(y0, y1) || y >= std::max(y0, y1)) {continue;}

      xs.push_back(x0 + (y - y0) * (x1 - x0) / (y1 - y0));
    }

    std::sort(xs.begin(), xs.end());

    for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
      int x_start = static_cast<int>(std::ceil(xs[k]));
      int x_end = static_cast<int>(std::ceil(xs[k + 1])) - 1;
      x_start = std::max(x_start, 0);
      x_end = std::min(x_end, map_width - 1);

      for (int x = x_start; x <= x_end; x++) {
        grid.data[static_cast<unsigned int>(y) * info.width +
          static_cast<unsigned int>(x)] = val;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// OPTIMIZED approach: scanline with all 3 optimizations applied
//   1. xs vector hoisted outside loop (reserve once, clear per scanline)
//   2. edges precomputed (y_lo, y_hi, x_at_ylo, inv_slope)
//   3. span filling uses std::fill instead of per-pixel loop
// (mirrors the final putFilled in vector_object_shapes.cpp)
// ---------------------------------------------------------------------------
static void fillScanlineOpt(
  OccGrid & grid,
  const std::vector<std::pair<double, double>> & pts,
  int8_t val)
{
  const std::size_t n = pts.size();
  if (n < 3) {return;}

  const auto & info = grid.info;
  const double origin_x = info.origin_x;
  const double origin_y = info.origin_y;
  const double res = info.resolution;

  std::vector<double> vx(n), vy(n);
  for (std::size_t i = 0; i < n; i++) {
    vx[i] = (pts[i].first - origin_x) / res - 0.5;
    vy[i] = (pts[i].second - origin_y) / res - 0.5;
  }

  int y_min = static_cast<int>(std::ceil(*std::min_element(vy.begin(), vy.end())));
  int y_max = static_cast<int>(std::floor(*std::max_element(vy.begin(), vy.end())));
  y_min = std::max(y_min, 0);
  y_max = std::min(y_max, static_cast<int>(info.height) - 1);

  // Optimization 2: precompute edge info
  struct EdgeInfo
  {
    double y_lo, y_hi, x_at_ylo, inv_slope;
  };
  std::vector<EdgeInfo> edges;
  edges.reserve(n);
  for (std::size_t i = 0; i < n; i++) {
    std::size_t j = (i + 1) % n;
    double y0 = vy[i], y1 = vy[j];
    double x0 = vx[i], x1 = vx[j];
    if (y0 == y1) {continue;}
    EdgeInfo e;
    if (y0 < y1) {
      e.y_lo = y0; e.y_hi = y1; e.x_at_ylo = x0;
    } else {
      e.y_lo = y1; e.y_hi = y0; e.x_at_ylo = x1;
    }
    e.inv_slope = (x1 - x0) / (y1 - y0);
    edges.push_back(e);
  }

  const int map_width = static_cast<int>(info.width);

  // Optimization 1: hoist xs outside the loop
  std::vector<double> xs;
  xs.reserve(edges.size());

  for (int y = y_min; y <= y_max; y++) {
    xs.clear();
    for (const auto & e : edges) {
      if (static_cast<double>(y) < e.y_lo || static_cast<double>(y) >= e.y_hi) {continue;}
      xs.push_back(e.x_at_ylo + (y - e.y_lo) * e.inv_slope);
    }

    std::sort(xs.begin(), xs.end());

    for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
      int x_start = static_cast<int>(std::ceil(xs[k]));
      int x_end = static_cast<int>(std::ceil(xs[k + 1])) - 1;
      x_start = std::max(x_start, 0);
      x_end = std::min(x_end, map_width - 1);
      if (x_start > x_end) {continue;}

      // Optimization 3: std::fill for contiguous span
      const unsigned int row_offset = static_cast<unsigned int>(y) * info.width;
      std::fill(
        grid.data.begin() + row_offset + x_start,
        grid.data.begin() + row_offset + x_end + 1,
        val);
    }
  }
}

// ---------------------------------------------------------------------------
// Benchmark helpers
// ---------------------------------------------------------------------------
struct Result
{
  double mean_us{0}, min_us{0}, max_us{0};
};

template<typename Fn>
Result bench(Fn fn, int iterations)
{
  std::vector<double> times;
  times.reserve(iterations);
  for (int i = 0; i < iterations; i++) {
    auto t0 = Clock::now();
    fn();
    auto t1 = Clock::now();
    times.push_back(
      static_cast<double>(std::chrono::duration_cast<us>(t1 - t0).count()));
  }
  Result r;
  r.mean_us = std::accumulate(times.begin(), times.end(), 0.0) / iterations;
  r.min_us = *std::min_element(times.begin(), times.end());
  r.max_us = *std::max_element(times.begin(), times.end());
  return r;
}

// ---------------------------------------------------------------------------
// Build test polygons (centered, fitting inside grid)
// ---------------------------------------------------------------------------
static std::vector<std::pair<double, double>> makePolygon(
  unsigned int grid_size,
  double fill_fraction,  // 0.0–1.0: how much of the grid the polygon covers
  int shape)             // 0=square, 1=triangle, 2=hexagon, 3=concave, 4=complex
{
  double cx = grid_size * 0.5;
  double cy = grid_size * 0.5;
  double r = grid_size * fill_fraction * 0.5;

  std::vector<std::pair<double, double>> pts;
  if (shape == 0) {  // square
    pts = {
      {cx - r, cy - r},
      {cx + r, cy - r},
      {cx + r, cy + r},
      {cx - r, cy + r}
    };
  } else if (shape == 1) {  // triangle
    pts = {
      {cx, cy + r},
      {cx - r, cy - r},
      {cx + r, cy - r}
    };
  } else if (shape == 2) {  // hexagon (6 vertices)
    for (int k = 0; k < 6; k++) {
      double a = k * M_PI / 3.0;
      pts.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
  } else if (shape == 3) {  // concave (arrow / chevron pointing right)
    double r2 = r * 0.4;
    pts = {
      {cx - r, cy - r2},
      {cx,     cy - r},
      {cx + r, cy},
      {cx,     cy + r},
      {cx - r, cy + r2},
      {cx - r2, cy}
    };
  } else {  // complex: 12-point star-like shape
    for (int k = 0; k < 12; k++) {
      double a = k * M_PI / 6.0;
      double rad = (k % 2 == 0) ? r : r * 0.5;
      pts.push_back({cx + rad * std::cos(a), cy + rad * std::sin(a)});
    }
  }
  return pts;
}

static void printHeader()
{
  std::cout
    << std::left << std::setw(42) << "Benchmark"
    << std::right << std::setw(12) << "old(µs)"
    << std::setw(12) << "new(µs)"
    << std::setw(12) << "opt(µs)"
    << std::setw(12) << "opencv(µs)"
    << std::setw(10) << "opt/old"
    << std::setw(10) << "opt/cv"
    << "\n"
    << std::string(110, '-') << "\n";
}

static void printRow(
  const std::string & label,
  const Result & r_old,
  const Result & r_new,
  const Result & r_opt,
  const Result * r_cv)  // nullptr if not available
{
  std::cout << std::left << std::setw(42) << label << std::right << std::fixed
            << std::setprecision(1)
            << std::setw(12) << r_old.mean_us
            << std::setw(12) << r_new.mean_us
            << std::setw(12) << r_opt.mean_us;
  if (r_cv) {
    std::cout << std::setw(12) << r_cv->mean_us;
  } else {
    std::cout << std::setw(12) << "N/A";
  }
  std::cout << std::setprecision(2)
            << std::setw(10) << (r_old.mean_us / r_opt.mean_us)
            << "x";
  if (r_cv) {
    std::cout << std::setw(10) << (r_opt.mean_us / r_cv->mean_us) << "x";
  } else {
    std::cout << std::setw(10) << "N/A";
  }
  std::cout << "\n";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  const int ITER = 300;
  const std::pair<unsigned int, double> GRID_SIZES[] = {
    {500,  0.85},
    {2000, 0.85}
  };
  // shape 0=square, 1=triangle, 2=hexagon, 3=concave, 4=complex
  const int SHAPES[] = {0, 1, 2, 3, 4};
  const char * SHAPE_NAMES[] = {"square", "triangle", "hexagon", "concave", "complex"};

  std::cout
    << "=== Polygon Rasterization Benchmark (Nav2 PR #6286) ===\n"
    << "Iterations per test: " << ITER << "\n"
    << "Polygon fill fraction: 85% of grid\n"
    << "Columns: old=bbox+isPointInside  new=scanline(unoptimized)"
    << "  opt=scanline(optimized)  opencv=cv::fillPoly\n"
    << "Each timing includes grid reset + fill (equivalent work).\n\n";

  printHeader();

  for (auto & [sz, fill] : GRID_SIZES) {
    for (int si = 0; si < 5; si++) {
      auto pts = makePolygon(sz, fill, SHAPES[si]);
      std::string tag =
        std::to_string(sz) + "x" + std::to_string(sz) +
        " " + SHAPE_NAMES[si];

      OccGrid g;
      g.reset(sz, sz, 1.0);

      // OLD
      auto r_old = bench(
        [&] {
          g.data.assign(g.data.size(), 0);
          fillOld(g, pts, 100);
        }, ITER);

      // NEW SCANLINE (unoptimized)
      auto r_new = bench(
        [&] {
          g.data.assign(g.data.size(), 0);
          fillScanline(g, pts, 100);
        }, ITER);

      // OPTIMIZED SCANLINE
      auto r_opt = bench(
        [&] {
          g.data.assign(g.data.size(), 0);
          fillScanlineOpt(g, pts, 100);
        }, ITER);

#ifdef HAVE_OPENCV
      // OPENCV
      std::vector<cv::Point> cv_pts;
      for (auto & p : pts) {
        cv_pts.push_back(cv::Point(static_cast<int>(p.first),
          static_cast<int>(p.second)));
      }
      std::vector<std::vector<cv::Point>> contours = {cv_pts};
      cv::Mat mat(sz, sz, CV_8SC1, cv::Scalar(0));

      auto r_cv = bench(
        [&] {
          mat.setTo(0);
          cv::fillPoly(mat, contours, cv::Scalar(100));
        }, ITER);

      printRow(tag, r_old, r_new, r_opt, &r_cv);
#else
      printRow(tag, r_old, r_new, r_opt, nullptr);
#endif
    }
    std::cout << "\n";
  }

#ifndef HAVE_OPENCV
  std::cout
    << "Note: OpenCV column shows N/A. Rebuild with -DHAVE_OPENCV and "
    << "$(pkg-config --cflags --libs opencv4) for full comparison.\n";
#endif

  return 0;
}
