// Copyright (c) 2026 Nav2 Contributors
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

// [AI-generated] This test file was implemented with AI assistance (Claude) and
// reviewed by the author.

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_smac_planner/obstacle_heuristic.hpp"
#include "nav2_smac_planner/types.hpp"

using nav2_smac_planner::ObstacleHeuristic;
using nav2_smac_planner::Coordinates;

static constexpr float REF_INF = std::numeric_limits<float>::infinity();

static void fillRandom(
  nav2_costmap_2d::Costmap2D & cm, unsigned int W, unsigned int H, unsigned int seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (unsigned int y = 0; y < H; ++y) {
    for (unsigned int x = 0; x < W; ++x) {
      double r = u(rng);
      unsigned char c = r < 0.20 ? 254 :
        (r < 0.35 ? static_cast<unsigned char>(1 + static_cast<int>(u(rng) * 251)) : 0);
      cm.setCost(x, y, c);
    }
  }
}

// Independent plain-Dijkstra oracle using the same cost model as the heuristic.
static std::vector<float> dijkstraReference(
  nav2_costmap_2d::Costmap2D * cm, unsigned int gx, unsigned int gy, float penalty)
{
  const unsigned int W = cm->getSizeInCellsX(), H = cm->getSizeInCellsY();
  const float SQRT2 = std::sqrt(2.0f);
  const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const bool DG[8] = {false, false, false, false, true, true, true, true};
  std::vector<float> dist(W * H, REF_INF);
  std::vector<char> closed(W * H, 0);
  using QE = std::pair<float, unsigned int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  unsigned int goal = gy * W + gx;
  dist[goal] = 0.0f;
  pq.push({0.0f, goal});
  while (!pq.empty()) {
    auto top = pq.top();
    pq.pop();
    unsigned int u = top.second;
    if (closed[u]) {continue;}
    closed[u] = 1;
    int ux = u % W, uy = u / W;
    for (int i = 0; i < 8; ++i) {
      int nx = ux + DX[i], ny = uy + DY[i];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(W) || ny >= static_cast<int>(H)) {continue;}
      unsigned int v = ny * W + nx;
      float cost = static_cast<float>(cm->getCost(v));
      if (cost >= 253.0f) {continue;}
      float ec = (DG[i] ? SQRT2 : 1.0f) * (1.0f + penalty * cost / 252.0f);
      float nd = top.first + ec;
      if (nd < dist[v]) {
        dist[v] = nd;
        pq.push({nd, v});
      }
    }
  }
  return dist;
}

static int compareFields(
  ObstacleHeuristic & h, const std::vector<float> & ref,
  unsigned int W, unsigned int H, double & maxd)
{
  int mism = 0;
  maxd = 0.0;
  for (unsigned int y = 0; y < H; ++y) {
    for (unsigned int x = 0; x < W; ++x) {
      float a = h.getIncrementalObstacleHeuristic(
        Coordinates(static_cast<float>(x), static_cast<float>(y), 0.0f));
      float b = ref[y * W + x];
      bool ai = std::isinf(a), bi = std::isinf(b);
      if (ai != bi) {mism++; continue;}
      if (ai && bi) {continue;}
      double d = std::fabs(static_cast<double>(a) - static_cast<double>(b));
      if (d > 1e-3) {mism++;}
      maxd = std::max(maxd, d);
    }
  }
  return mism;
}

// Incremental update must equal a full recompute (and an independent Dijkstra),
// while being cheaper for small changes.
TEST(ObstacleHeuristicIncremental, matches_full_recompute_and_dijkstra)
{
  const unsigned int W = 150, H = 150;
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  nav2_costmap_2d::Costmap2D raw(W, H, 0.05, 0.0, 0.0, 0);
  fillRandom(raw, W, H, 42);
  *costmap = raw;
  const unsigned int gx = W / 2, gy = H / 2;
  costmap->setCost(gx, gy, 0);

  const float penalty = 2.0f;
  const bool quad = false;

  ObstacleHeuristic inc;
  inc.resetIncrementalObstacleHeuristic(costmap_ros, gx, gy, penalty, quad);

  // sanity: the from-scratch incremental field equals an independent Dijkstra
  {
    auto ref0 = dijkstraReference(costmap, gx, gy, penalty);
    double md0 = 0.0;
    EXPECT_EQ(compareFields(inc, ref0, W, H, md0), 0) << "engine != Dijkstra on reset";
  }

  // mutate K cells on the live costmap
  std::mt19937 rng(9001);
  std::uniform_int_distribution<unsigned int> dx(0, W - 1), dy(0, H - 1), dc(0, 2);
  const int K = 30;
  for (int k = 0; k < K; ++k) {
    unsigned int x = dx(rng), y = dy(rng);
    if (x == gx && y == gy) {continue;}
    unsigned char nv = dc(rng) == 0 ? 254 : (dc(rng) == 1 ? 120 : 0);
    costmap->setCost(x, y, nv);
  }

  auto t0 = std::chrono::steady_clock::now();
  unsigned int changed = inc.updateIncrementalObstacleHeuristic(penalty, quad);
  auto t1 = std::chrono::steady_clock::now();

  ObstacleHeuristic full;
  auto t2 = std::chrono::steady_clock::now();
  full.resetIncrementalObstacleHeuristic(costmap_ros, gx, gy, penalty, quad);
  auto t3 = std::chrono::steady_clock::now();

  auto ref = dijkstraReference(costmap, gx, gy, penalty);
  double md_inc = 0.0, md_full = 0.0;
  int mism_inc = compareFields(inc, ref, W, H, md_inc);
  int mism_full = compareFields(full, ref, W, H, md_full);

  double inc_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  double full_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
  printf(
    "[incremental] map=%ux%u changed=%u  incr=%.3f ms  full=%.3f ms  "
    "speedup=%.1fx  maxdiff(inc)=%.5f mism(inc)=%d mism(full)=%d\n",
    W, H, changed, inc_ms, full_ms, inc_ms > 1e-6 ? full_ms / inc_ms : 0.0,
    md_inc, mism_inc, mism_full);

  EXPECT_GT(changed, 0u);
  EXPECT_EQ(mism_inc, 0) << "incremental update != Dijkstra ground truth";
  EXPECT_EQ(mism_full, 0) << "full recompute != Dijkstra ground truth";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
