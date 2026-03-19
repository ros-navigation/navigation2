// Copyright (c) 2026, Nav2 Contributors
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

// Benchmark: Before (standard msg + convert_to_custom) vs After (TypeAdapter zero-copy)
// Measures end-to-end publish→callback latency for costmap subscriptions at
// two map sizes: 200x200 (local costmap) and 1000x1000 (global costmap).

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_type_adapter.hpp"
#include "nav2_msgs/msg/costmap.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static nav2_costmap_2d::Costmap2DStamped make_stamped(
  unsigned int size_x, unsigned int size_y)
{
  nav2_costmap_2d::Costmap2DStamped s;
  s.header.frame_id = "map";
  s.metadata.resolution = 0.05f;
  s.metadata.size_x = size_x;
  s.metadata.size_y = size_y;
  s.costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    size_x, size_y, 0.05, 0.0, 0.0);
  auto * data = s.costmap->getCharMap();
  for (unsigned int i = 0; i < size_x * size_y; ++i) {
    data[i] = static_cast<unsigned char>(i % 254 + 1);
  }
  return s;
}

static nav2_msgs::msg::Costmap make_msg(unsigned int size_x, unsigned int size_y)
{
  nav2_msgs::msg::Costmap msg;
  msg.header.frame_id = "map";
  msg.metadata.resolution = 0.05f;
  msg.metadata.size_x = size_x;
  msg.metadata.size_y = size_y;
  msg.data.resize(size_x * size_y);
  for (unsigned int i = 0; i < size_x * size_y; ++i) {
    msg.data[i] = static_cast<uint8_t>(i % 254 + 1);
  }
  return msg;
}

static void print_stats(
  const std::string & label,
  const std::vector<double> & samples_us)
{
  double sum = std::accumulate(samples_us.begin(), samples_us.end(), 0.0);
  double mean = sum / static_cast<double>(samples_us.size());
  double mn = *std::min_element(samples_us.begin(), samples_us.end());
  double mx = *std::max_element(samples_us.begin(), samples_us.end());
  std::printf(
    "  %-44s  mean=%7.1f us  min=%7.1f us  max=%7.1f us\n",
    label.c_str(), mean, mn, mx);
}

// ---------------------------------------------------------------------------
// After: TypeAdapter (zero-copy intra-process, no conversion)
// ---------------------------------------------------------------------------

static std::vector<double> bench_after(
  unsigned int size_x, unsigned int size_y, int n_iters)
{
  using Adapter = rclcpp::TypeAdapter<
    nav2_costmap_2d::Costmap2DStamped, nav2_msgs::msg::Costmap>;

  auto node = rclcpp::Node::make_shared(
    "bench_after",
    rclcpp::NodeOptions().use_intra_process_comms(true));

  std::atomic<int64_t> recv_ns{0};
  auto sub = node->create_subscription<Adapter>(
    "costmap_after", rclcpp::SensorDataQoS(),
    [&recv_ns](std::shared_ptr<nav2_costmap_2d::Costmap2DStamped>/*msg*/) {
      recv_ns.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
        std::memory_order_relaxed);
    });

  auto pub = node->create_publisher<Adapter>(
    "costmap_after", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // Warm-up
  for (int i = 0; i < 10; ++i) {
    auto stamped = std::make_unique<nav2_costmap_2d::Costmap2DStamped>(
      make_stamped(size_x, size_y));
    pub->publish(std::move(stamped));
    exec.spin_some();
  }

  std::vector<double> samples;
  samples.reserve(n_iters);

  for (int i = 0; i < n_iters; ++i) {
    auto stamped = std::make_unique<nav2_costmap_2d::Costmap2DStamped>(
      make_stamped(size_x, size_y));
    recv_ns.store(0, std::memory_order_relaxed);
    int64_t send_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    pub->publish(std::move(stamped));
    exec.spin_some(std::chrono::milliseconds(10));
    int64_t r = recv_ns.load(std::memory_order_relaxed);
    if (r > 0) {
      samples.push_back(static_cast<double>(r - send_ns) / 1000.0);
    }
  }
  return samples;
}

// ---------------------------------------------------------------------------
// Before: Standard msg + convert_to_custom() in callback (old code path)
// The old CostmapSubscriber received nav2_msgs::msg::Costmap and called
// processCurrentCostmapMsg() which did the equivalent of convert_to_custom()
// on every message to produce a Costmap2D.
// ---------------------------------------------------------------------------

static std::vector<double> bench_before(
  unsigned int size_x, unsigned int size_y, int n_iters)
{
  using Adapter = rclcpp::TypeAdapter<
    nav2_costmap_2d::Costmap2DStamped, nav2_msgs::msg::Costmap>;

  auto node = rclcpp::Node::make_shared(
    "bench_before",
    rclcpp::NodeOptions().use_intra_process_comms(true));

  std::atomic<int64_t> recv_ns{0};
  auto sub = node->create_subscription<nav2_msgs::msg::Costmap>(
    "costmap_before", rclcpp::SensorDataQoS(),
    [&recv_ns](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
      // Simulate the old processCurrentCostmapMsg(): convert msg → Costmap2D
      nav2_costmap_2d::Costmap2DStamped dst;
      Adapter::convert_to_custom(*msg, dst);
      recv_ns.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
        std::memory_order_relaxed);
    });

  auto pub = node->create_publisher<nav2_msgs::msg::Costmap>(
    "costmap_before", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // Warm-up
  for (int i = 0; i < 10; ++i) {
    pub->publish(make_msg(size_x, size_y));
    exec.spin_some();
  }

  std::vector<double> samples;
  samples.reserve(n_iters);

  for (int i = 0; i < n_iters; ++i) {
    auto msg = std::make_unique<nav2_msgs::msg::Costmap>(make_msg(size_x, size_y));
    recv_ns.store(0, std::memory_order_relaxed);
    int64_t send_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    pub->publish(std::move(msg));
    exec.spin_some(std::chrono::milliseconds(10));
    int64_t r = recv_ns.load(std::memory_order_relaxed);
    if (r > 0) {
      samples.push_back(static_cast<double>(r - send_ns) / 1000.0);
    }
  }
  return samples;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const int N = 1000;
  const struct { unsigned int x, y; const char * label; } sizes[] = {
    {200, 200, "200x200 (local costmap, ~40 KB)"},
    {1000, 1000, "1000x1000 (global costmap, ~1 MB)"},
  };

  std::printf("\n=== Costmap TypeAdapter Benchmark: Before vs After ===\n");
  std::printf("N=%d iterations per configuration (10 warm-up excluded)\n\n", N);
  std::printf(
    "Before: standard msg subscription + convert_to_custom() in callback\n"
    "After:  TypeAdapter subscription (zero-copy intra-process, no conversion)\n\n");

  for (auto & s : sizes) {
    std::printf("[Map size: %s]\n", s.label);

    auto before = bench_before(s.x, s.y, N);
    if (!before.empty()) {
      print_stats("Before (msg + convert_to_custom)", before);
    } else {
      std::printf("  Before: NO SAMPLES\n");
    }

    auto after = bench_after(s.x, s.y, N);
    if (!after.empty()) {
      print_stats("After  (TypeAdapter, zero-copy)", after);
    } else {
      std::printf("  After: NO SAMPLES\n");
    }

    std::printf("\n");
  }

  rclcpp::shutdown();
  return 0;
}
