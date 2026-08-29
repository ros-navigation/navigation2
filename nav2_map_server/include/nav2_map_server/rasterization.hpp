// Copyright (c) 2024 Nav2 Authors
// SPDX-License-Identifier: Apache-2.0
//
// Ported from OpenCV's modules/imgproc/src/drawing.cpp
// Original copyright: Intel Corporation, Willow Garage, Itseez (Intel License / BSD)
// See: https://github.com/opencv/opencv/blob/4.x/modules/imgproc/src/drawing.cpp
//
// Adapted to Nav2 types — no cv::Mat, no cv::Point, no OpenCV dependency.
// Algorithm is a 1:1 port of OpenCV; only data-type substitutions are made:
//   cv::Mat           → Grid (int8_t* data + width + height)
//   cv::Point2l/int64 → Pt2l { int64_t x, y }  (fixed-point, XY_SHIFT=16)
//   cv::Scalar color  → int8_t fill_value
//   img.ptr(y)        → grid.data + y * grid.width
//   ICV_HLINE macro   → hline() helper (std::fill on int8_t span)

#ifndef NAV2_MAP_SERVER__RASTERIZATION_HPP_
#define NAV2_MAP_SERVER__RASTERIZATION_HPP_

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace nav2_map_server
{

// ============================================================================
// Constants — same as OpenCV drawing.cpp enum (line ~50 in OpenCV source)
// ============================================================================
static constexpr int XY_SHIFT = 16;                  // fractional bits
static constexpr int64_t XY_ONE = 1LL << XY_SHIFT;  // 1.0 in fixed-point

// ============================================================================
// Grid — Nav2 equivalent of cv::Mat (1-channel int8_t)
// Wraps the flat data[] array of an OccupancyGrid together with dimensions.
// ============================================================================
struct Grid
{
  int8_t * data;
  int width;
  int height;
  int8_t fill_value = 0;  ///< value written by hline/put_point (OVERLAY_SEQ)

  /// Optional per-cell writer for non-OVERLAY_SEQ cases.
  /// Signature: void(int x, int y). When empty, direct write is used.
  std::function<void(int, int)> write_cell;

  inline int8_t & at(int x, int y) {return data[y * width + x];}
  inline int8_t * row(int y) {return data + y * width;}
  inline bool in_bounds(int x, int y) const
  {
    return x >= 0 && x < width && y >= 0 && y < height;
  }
};

// ============================================================================
// Pt2l — Nav2 equivalent of cv::Point2l (fixed-point 2-D point, int64)
// ============================================================================
struct Pt2l
{
  int64_t x, y;
};

// ============================================================================
// PolyEdge — Nav2 equivalent of cv::PolyEdge (drawing.cpp ~line 54)
// Represents one non-horizontal edge in the active-edge table.
//   y0, y1 : integer scanline range [y0, y1)
//   x      : current X in fixed-point (XY_SHIFT fractional bits)
//   dx     : per-scanline X increment in fixed-point
//   next   : intrusive linked-list pointer used by FillEdgeCollection
// ============================================================================
struct PolyEdge
{
  int y0, y1;
  int64_t x, dx;
  PolyEdge * next;

  PolyEdge()
  : y0(0), y1(0), x(0), dx(0), next(nullptr) {}
};

// ============================================================================
// hline — Nav2 equivalent of ICV_HLINE macro (drawing.cpp ~line 1098)
// Fills grid row y from column x1 to x2 (inclusive) with value.
// ============================================================================
inline void hline(Grid & g, int y, int x1, int x2, int8_t value)
{
  // x1/x2 are already clipped to [0, width-1] by the caller.
  if (g.write_cell) {
    for (int x = x1; x <= x2; x++) {
      g.write_cell(x, y);
    }
  } else {
    int8_t * r = g.row(y);
    std::fill(r + x1, r + x2 + 1, value);
  }
}

// ============================================================================
// put_point — Nav2 equivalent of ICV_PUT_POINT (draw a single pixel).
// ============================================================================
inline void put_point(Grid & g, int x, int y, int8_t value)
{
  if (!g.in_bounds(x, y)) {return;}
  if (g.write_cell) {g.write_cell(x, y);} else {g.at(x, y) = value;}
}

// ============================================================================
// Line2 — Nav2 equivalent of OpenCV's static Line2() (drawing.cpp ~line 643)
// Draws a 1-pixel-wide line between two fixed-point endpoints using
// incremental integer arithmetic.  Inputs are already in fixed-point
// (integer pixel coordinates * XY_ONE).
// ============================================================================
inline void line2(Grid & g, Pt2l pt1, Pt2l pt2, int8_t value)
{
  // OpenCV Line2: clip to [0, size<<XY_SHIFT) first.
  // We skip the clip call and instead guard each put_point.

  int64_t dx = pt2.x - pt1.x;
  int64_t dy = pt2.y - pt1.y;

  // abs without branching: j = sign-extension trick from OpenCV
  int64_t j = dx < 0 ? -1 : 0;
  int64_t ax = (dx ^ j) - j;   // ax = abs(dx)
  int64_t i = dy < 0 ? -1 : 0;
  int64_t ay = (dy ^ i) - i;   // ay = abs(dy)

  int ecount;
  int64_t x_step, y_step;

  if (ax > ay) {
    // X-major: step one pixel in X per iteration
    dy = (dy ^ j) - j;  // dy = abs(dy) oriented toward pt2
    // Swap pt1/pt2 so pt1.x < pt2.x
    pt1.x ^= pt2.x & j; pt2.x ^= pt1.x & j; pt1.x ^= pt2.x & j;
    pt1.y ^= pt2.y & j; pt2.y ^= pt1.y & j; pt1.y ^= pt2.y & j;

    x_step = XY_ONE;
    y_step = dy * XY_ONE / (ax | 1);
    ecount = static_cast<int>((pt2.x - pt1.x) >> XY_SHIFT);
  } else {
    // Y-major: step one pixel in Y per iteration
    dx = (dx ^ i) - i;
    pt1.x ^= pt2.x & i; pt2.x ^= pt1.x & i; pt1.x ^= pt2.x & i;
    pt1.y ^= pt2.y & i; pt2.y ^= pt1.y & i; pt1.y ^= pt2.y & i;

    x_step = dx * XY_ONE / (ay | 1);
    y_step = XY_ONE;
    ecount = static_cast<int>((pt2.y - pt1.y) >> XY_SHIFT);
  }

  // Round to nearest pixel at start
  pt1.x += XY_ONE >> 1;
  pt1.y += XY_ONE >> 1;

  // Draw the endpoint explicitly (matching OpenCV's ICV_PUT_POINT at end)
  {
    int ex = static_cast<int>((pt2.x + (XY_ONE >> 1)) >> XY_SHIFT);
    int ey = static_cast<int>((pt2.y + (XY_ONE >> 1)) >> XY_SHIFT);
    put_point(g, ex, ey, value);
  }

  if (ax > ay) {
    // X-major inner loop
    int64_t px = pt1.x >> XY_SHIFT;
    for (; ecount >= 0; px++, pt1.y += y_step, ecount--) {
      put_point(g, static_cast<int>(px), static_cast<int>(pt1.y >> XY_SHIFT), value);
    }
  } else {
    // Y-major inner loop
    int64_t py = pt1.y >> XY_SHIFT;
    for (; ecount >= 0; py++, pt1.x += x_step, ecount--) {
      put_point(g, static_cast<int>(pt1.x >> XY_SHIFT), static_cast<int>(py), value);
    }
  }
}

// ============================================================================
// collect_poly_edges — Nav2 equivalent of OpenCV's CollectPolyEdges()
// (drawing.cpp ~line 1273)
//
// For each non-horizontal edge of the polygon, computes:
//   y0, y1   : integer scanline range
//   x        : starting X in fixed-point at scanline y0
//   dx       : per-scanline X increment in fixed-point
// and draws the border line via line2().
//
// Inputs: pts[] are already in fixed-point (world-to-pixel * XY_ONE).
// shift=0 means pts are integers (no sub-pixel).  For our use shift=0.
// ============================================================================
inline void collect_poly_edges(
  Grid & g,
  const std::vector<Pt2l> & pts,
  std::vector<PolyEdge> & edges,
  bool draw_border,
  int8_t value)
{
  const int count = static_cast<int>(pts.size());
  if (count < 1) {return;}

  edges.reserve(edges.size() + count);

  // OpenCV uses shift and offset; we use shift=0, offset=(0,0).
  // With shift=0: pt.x <<= (XY_SHIFT - 0) = XY_SHIFT, pt.y >>= 0 = pt.y>>0.
  // But our pts already store integer pixel coords, so we pre-scale here.

  // Start from the last vertex (wraps around to form a closed polygon).
  Pt2l p0 = pts[count - 1];
  // Scale to fixed-point: x gets XY_SHIFT fractional bits, y is integer.
  p0.x = p0.x << XY_SHIFT;   // integer pixel → fixed-point
  p0.y = p0.y;                // already integer scanline

  for (int i = 0; i < count; i++) {
    Pt2l p1 = pts[i];
    // Input pts are already in fixed-point (XY_SHIFT = 16), no shift needed.

    // Draw the border segment using Line2 algorithm (equivalent to
    // OpenCV's Line() call inside CollectPolyEdges for line_type=8).
    if (draw_border) {
      // Convert to rounded integer coords for line drawing.
      Pt2l t0, t1;
      t0.x = (p0.x + (XY_ONE >> 1)) >> XY_SHIFT;
      t0.y = p0.y;
      t1.x = (p1.x + (XY_ONE >> 1)) >> XY_SHIFT;
      t1.y = p1.y;
      // Call line2 with both endpoints expressed in fixed-point
      Pt2l fp0{t0.x << XY_SHIFT, t0.y << XY_SHIFT};
      Pt2l fp1{t1.x << XY_SHIFT, t1.y << XY_SHIFT};
      line2(g, fp0, fp1, value);
    }

    // Skip horizontal edges — they never intersect scanlines.
    if (p0.y == p1.y) {
      p0 = p1;
      continue;
    }

    // Build PolyEdge. Equivalent to drawing.cpp lines 1330-1343.
    // dx = (x1 - x0) / (y1 - y0)  in fixed-point
    PolyEdge edge;
    edge.dx = (p1.x - p0.x) / (p1.y - p0.y);

    if (p0.y < p1.y) {
      edge.y0 = static_cast<int>(p0.y);
      edge.y1 = static_cast<int>(p1.y);
      edge.x = p0.x;
    } else {
      edge.y0 = static_cast<int>(p1.y);
      edge.y1 = static_cast<int>(p0.y);
      edge.x = p1.x;
    }
    edges.push_back(edge);
    p0 = p1;
  }
}

// ============================================================================
// CmpEdges — comparator for initial sort in FillEdgeCollection
// (drawing.cpp ~line 1347).  Primary: y0 ascending; secondary: x ascending.
// ============================================================================
struct CmpEdges
{
  bool operator()(const PolyEdge & a, const PolyEdge & b) const
  {
    if (a.y0 != b.y0) {return a.y0 < b.y0;}
    if (a.x != b.x) {return a.x < b.x;}
    return a.dx < b.dx;
  }
};

// ============================================================================
// fill_edge_collection — Nav2 equivalent of OpenCV's FillEdgeCollection()
// (drawing.cpp ~line 1358).
//
// Implements the classic scanline Active Edge Table (AET) algorithm:
//  1. Sort all edges by y0 (start scanline).
//  2. Sweep y from y_min to y_max.
//  3. Maintain a linked list of currently-active edges, sorted by x.
//  4. At each scanline, fill between pairs of active-edge x values.
//  5. Advance each active edge: edge.x += edge.dx  (one addition — no multiply).
//  6. Bubble-sort the AET to maintain x order after each advance.
// ============================================================================
inline void fill_edge_collection(
  Grid & g,
  std::vector<PolyEdge> & edges,
  int8_t value)
{
  const int total = static_cast<int>(edges.size());
  if (total < 2) {return;}

  // Determine overall y bounds and x bounds for early-exit check.
  int y_min = INT_MAX, y_max = INT_MIN;
  int64_t x_min = INT64_MAX, x_max = INT64_MIN;

  for (int i = 0; i < total; i++) {
    const PolyEdge & e = edges[i];
    // OpenCV asserts e.y0 < e.y1; we skip degenerate edges silently.
    if (e.y0 >= e.y1) {continue;}
    int64_t x_end = e.x + static_cast<int64_t>(e.y1 - e.y0) * e.dx;
    y_min = std::min(y_min, e.y0);
    y_max = std::max(y_max, e.y1);
    x_min = std::min(x_min, std::min(e.x, x_end));
    x_max = std::max(x_max, std::max(e.x, x_end));
  }

  // Off-screen check (matching drawing.cpp line 1388).
  if (y_max <= 0 || y_min >= g.height ||
    x_max < 0 || x_min >= (static_cast<int64_t>(g.width) << XY_SHIFT))
  {
    return;
  }

  // Sort edges by (y0, x, dx).
  std::sort(edges.begin(), edges.end(), CmpEdges());

  // Sentinel at end so the AET insertion loop terminates cleanly.
  PolyEdge sentinel;
  sentinel.y0 = INT_MAX;
  edges.push_back(sentinel);

  // AET head (dummy node).
  PolyEdge tmp;
  tmp.next = nullptr;

  int i = 0;  // index into sorted edges[] for next insertion
  PolyEdge * e_next = &edges[i];

  y_max = std::min(y_max, g.height);  // clamp to grid height

  // Sweep scanlines from y_min to y_max-1.
  for (int y = e_next->y0; y < y_max; y++) {
    PolyEdge * last;
    PolyEdge * prelast;
    PolyEdge * keep_prelast;
    int draw = 0;
    const bool clipline = (y < 0);

    prelast = &tmp;
    last = tmp.next;

    // Process active list: insert new edges at y0==y, remove edges at y1==y.
    while (last || e_next->y0 == y) {
      if (last && last->y1 == y) {
        // Edge has reached its end: remove from AET.
        prelast->next = last->next;
        last = last->next;
        continue;
      }
      keep_prelast = prelast;

      if (last && (e_next->y0 > y || last->x < e_next->x)) {
        // Advance along existing AET.
        prelast = last;
        last = last->next;
      } else if (i < total) {
        // Insert next edge from sorted list into AET at correct x position.
        prelast->next = e_next;
        e_next->next = last;
        prelast = e_next;
        e_next = &edges[++i];
      } else {
        break;
      }

      if (draw) {
        if (!clipline) {
          // Fill between keep_prelast->x and prelast->x.
          // delta = XY_ONE-1 provides the "round toward the interior" rounding
          // that OpenCV uses (drawing.cpp line 1448-1454).
          const int64_t delta = XY_ONE - 1;
          int x1, x2;
          if (keep_prelast->x > prelast->x) {
            x1 = static_cast<int>((prelast->x + delta) >> XY_SHIFT);
            x2 = static_cast<int>(keep_prelast->x >> XY_SHIFT);
          } else {
            x1 = static_cast<int>((keep_prelast->x + delta) >> XY_SHIFT);
            x2 = static_cast<int>(prelast->x >> XY_SHIFT);
          }
          if (x1 < g.width && x2 >= 0) {
            x1 = std::max(x1, 0);
            x2 = std::min(x2, g.width - 1);
            hline(g, y, x1, x2, value);
          }
        }
        // Advance the two edge x values (one addition each — no multiply).
        keep_prelast->x += keep_prelast->dx;
        prelast->x += prelast->dx;
      }
      draw ^= 1;
    }

    // Bubble-sort the AET by x to maintain left-to-right order.
    // OpenCV uses a partial bubble sort that stops early (drawing.cpp ~1476).
    PolyEdge * keep_last = nullptr;
    do {
      prelast = &tmp;
      last = tmp.next;
      PolyEdge * last_exchange = nullptr;

      while (last != keep_last && last->next != nullptr) {
        PolyEdge * temp_edge = last->next;
        if (last->x > temp_edge->x) {
          prelast->next = temp_edge;
          last->next = temp_edge->next;
          temp_edge->next = last;
          prelast = temp_edge;
          last_exchange = prelast;
        } else {
          prelast = last;
          last = temp_edge;
        }
      }
      if (last_exchange == nullptr) {break;}
      keep_last = last_exchange;
    } while (keep_last != tmp.next && keep_last != &tmp);
  }
}

// ============================================================================
// fill_polygon — public API
// Draws a filled polygon onto grid g.
// pts[]: integer pixel coordinates (NOT fixed-point; we scale internally).
// Equivalent to calling CollectPolyEdges + FillEdgeCollection in OpenCV.
// ============================================================================
inline void fill_polygon(
  Grid & g,
  const std::vector<Pt2l> & pts,
  int8_t value)
{
  if (pts.size() < 3) {return;}
  std::vector<PolyEdge> edges;
  // draw_border=false: CollectPolyEdges draws the border as part of the
  // scanline setup; we omit it here because putBorders handles borders.
  collect_poly_edges(g, pts, edges, /*draw_border=*/ false, value);
  fill_edge_collection(g, edges, value);
}

// ============================================================================
// draw_polyline — public API
// Draws polygon borders onto grid g (open or closed).
// pts[]: integer pixel coordinates.
// Equivalent to OpenCV's PolyLine() with thickness=1, line_type=8, shift=0.
// ============================================================================
inline void draw_polyline(
  Grid & g,
  const std::vector<Pt2l> & pts,
  bool closed,
  int8_t value)
{
  const int count = static_cast<int>(pts.size());
  if (count < 2) {return;}

  const int start = closed ? count - 1 : 0;
  Pt2l p0 = pts[start];

  for (int i = (closed ? 0 : 1); i < count; i++) {
    Pt2l p1 = pts[i];
    // Convert integer pixel coords to fixed-point for Line2.
    // Input pts are already in fixed-point (XY_SHIFT = 16), no shift needed.
    Pt2l fp0 = p0;
    Pt2l fp1 = p1;
    line2(g, fp0, fp1, value);
    p0 = p1;
  }
}

// ============================================================================
// draw_circle — Nav2 equivalent of OpenCV's Circle() (drawing.cpp ~line 1512)
//
// Integer-only Bresenham/midpoint circle algorithm with 8-way symmetry.
// Variables (err, dx, dy, plus, minus) are identical to OpenCV's.
//
// If filled=true, each step fills two horizontal spans (y±dy rows and y±dx
// rows), equivalent to ICV_HLINE in OpenCV's filled-circle branch.
// If filled=false, plots 8 symmetric border pixels per step.
// ============================================================================
inline void draw_circle(
  Grid & g,
  int cx, int cy, int radius,
  bool filled,
  int8_t value)
{
  // OpenCV initialisation (drawing.cpp line 1518):
  //   err=0, dx=radius, dy=0, plus=1, minus=(radius<<1)-1
  int64_t err = 0;
  int64_t dx = radius;
  int64_t dy = 0;
  int64_t plus = 1;
  int64_t minus = (radius << 1) - 1;

  // 'inside': true if the entire circle fits within the grid — allows
  // skipping per-pixel bounds checks for the common case.
  const bool inside =
    cx >= radius && cx < g.width - radius &&
    cy >= radius && cy < g.height - radius;

  // Helper: fill a horizontal span clamped to grid bounds.
  auto do_hline = [&](int y, int64_t xl, int64_t xr) {
      if (y < 0 || y >= g.height) {return;}
      int x1 = static_cast<int>(std::max(xl, int64_t(0)));
      int x2 = static_cast<int>(std::min(xr, int64_t(g.width - 1)));
      if (x1 <= x2) {hline(g, y, x1, x2, value);}
    };

  // Helper: plot a single pixel with bounds check.
  auto do_point = [&](int64_t x, int64_t y) {
      put_point(g, static_cast<int>(x), static_cast<int>(y), value);
    };

  while (dx >= dy) {
    // The four symmetric row/column indices (matching OpenCV line 1528):
    int64_t y11 = cy - dy, y12 = cy + dy;  // rows at ±dy
    int64_t y21 = cy - dx, y22 = cy + dx;  // rows at ±dx
    int64_t x11 = cx - dx, x12 = cx + dx;  // columns at ±dx
    int64_t x21 = cx - dy, x22 = cx + dy;  // columns at ±dy

    if (inside) {
      // Fast path: no bounds checks needed.
      if (!filled) {
        // 8 border pixels (OpenCV's ICV_PUT_POINT calls in inside branch).
        g.at(static_cast<int>(x11), static_cast<int>(y11)) = value;
        g.at(static_cast<int>(x11), static_cast<int>(y12)) = value;
        g.at(static_cast<int>(x12), static_cast<int>(y11)) = value;
        g.at(static_cast<int>(x12), static_cast<int>(y12)) = value;
        g.at(static_cast<int>(x21), static_cast<int>(y21)) = value;
        g.at(static_cast<int>(x21), static_cast<int>(y22)) = value;
        g.at(static_cast<int>(x22), static_cast<int>(y21)) = value;
        g.at(static_cast<int>(x22), static_cast<int>(y22)) = value;
      } else {
        // 4 horizontal spans (OpenCV's ICV_HLINE calls in inside+fill branch).
        hline(
          g, static_cast<int>(y11), static_cast<int>(x11),
          static_cast<int>(x12), value);
        hline(
          g, static_cast<int>(y12), static_cast<int>(x11),
          static_cast<int>(x12), value);
        hline(
          g, static_cast<int>(y21), static_cast<int>(x21),
          static_cast<int>(x22), value);
        hline(
          g, static_cast<int>(y22), static_cast<int>(x21),
          static_cast<int>(x22), value);
      }
    } else if (x11 < g.width && x12 >= 0 && y21 < g.height && y22 >= 0) {
      // Partial visibility — clip each span/point.
      if (filled) {
        int64_t fx11 = std::max(x11, int64_t(0));
        int64_t fx12 = std::min(x12, int64_t(g.width - 1));
        do_hline(static_cast<int>(y11), fx11, fx12);
        do_hline(static_cast<int>(y12), fx11, fx12);

        if (x21 < g.width && x22 >= 0) {
          int64_t fx21 = std::max(x21, int64_t(0));
          int64_t fx22 = std::min(x22, int64_t(g.width - 1));
          do_hline(static_cast<int>(y21), fx21, fx22);
          do_hline(static_cast<int>(y22), fx21, fx22);
        }
      } else {
        // Border pixels with individual bounds checks.
        if (y11 >= 0 && y11 < g.height) {
          if (x11 >= 0) {do_point(x11, y11);}
          if (x12 < g.width) {do_point(x12, y11);}
        }
        if (y12 >= 0 && y12 < g.height) {
          if (x11 >= 0) {do_point(x11, y12);}
          if (x12 < g.width) {do_point(x12, y12);}
        }
        if (x21 < g.width && x22 >= 0) {
          if (y21 >= 0 && y21 < g.height) {
            if (x21 >= 0) {do_point(x21, y21);}
            if (x22 < g.width) {do_point(x22, y21);}
          }
          if (y22 >= 0 && y22 < g.height) {
            if (x21 >= 0) {do_point(x21, y22);}
            if (x22 < g.width) {do_point(x22, y22);}
          }
        }
      }
    }

    // Advance the Bresenham variables (drawing.cpp lines 1642-1650):
    //   dy++; err += plus; plus += 2;
    //   mask = (err <= 0) - 1;       // all-ones if err>0, else 0
    //   err  -= minus & mask;
    //   dx   += mask;                // dx-- if err>0
    //   minus -= mask & 2;           // minus -= 2 if err>0
    dy++;
    err += plus;
    plus += 2;

    int64_t mask = (err <= 0) - 1;
    err -= minus & mask;
    dx += mask;
    minus -= mask & 2;
  }
}

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__RASTERIZATION_HPP_
