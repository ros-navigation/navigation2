// Copyright (c) 2026 wheel.me
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace
{

geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

const std::vector<geometry_msgs::msg::Point> small_footprint{
  makePoint(0.3, 0.3), makePoint(0.3, -0.3), makePoint(-0.3, -0.3), makePoint(-0.3, 0.3)};

const std::vector<geometry_msgs::msg::Point> large_footprint{
  makePoint(0.6, 0.0), makePoint(0.4, -0.4), makePoint(0.0, -0.6), makePoint(-0.4, -0.4),
  makePoint(-0.6, 0.0), makePoint(-0.4, 0.4), makePoint(0.0, 0.6), makePoint(0.4, 0.4)};

bool belongsTo(
  const geometry_msgs::msg::Point & point,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  for (const auto & candidate : footprint) {
    if (candidate.x == point.x && candidate.y == point.y && candidate.z == point.z) {
      return true;
    }
  }
  return false;
}

}  // namespace

// A footprint read must stay intact while another thread replaces the stored footprint.
TEST(LayeredCostmapFootprint, concurrentSetAndGetFootprint)
{
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.setFootprint(small_footprint);

  std::atomic<bool> stop{false};
  std::atomic<size_t> bad_reads{0};
  std::atomic<size_t> reads{0};

  std::thread writer([&]() {
      bool use_large = true;
      while (!stop.load()) {
        layers.setFootprint(use_large ? large_footprint : small_footprint);
        use_large = !use_large;
      }
    });

  std::thread reader([&]() {
      while (!stop.load()) {
        for (const auto & point : layers.getFootprint()) {
          if (!belongsTo(point, small_footprint) && !belongsTo(point, large_footprint)) {
            ++bad_reads;
          }
        }
        const size_t size = layers.getFootprint().size();
        if (size != small_footprint.size() && size != large_footprint.size()) {
          ++bad_reads;
        }
        ++reads;
      }
    });

  std::this_thread::sleep_for(std::chrono::seconds(1));
  stop.store(true);
  writer.join();
  reader.join();

  EXPECT_GT(reads.load(), 0u);
  EXPECT_EQ(bad_reads.load(), 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
