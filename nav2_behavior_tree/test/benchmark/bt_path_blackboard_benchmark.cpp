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

#include <benchmark/benchmark.h>
#include <memory>
#include "behaviortree_cpp/blackboard.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"

static nav_msgs::msg::Path make_path(size_t n_poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(n_poses);
  for (size_t i = 0; i < n_poses; ++i) {
    path.poses[i].pose.position.x = static_cast<double>(i) * 0.05;
  }
  return path;
}

// BEFORE: blackboard stores raw Path — get copies the entire vector
static void BM_BlackboardGetPath_Copy(benchmark::State & state)
{
  auto bb = BT::Blackboard::create();
  bb->set("path", make_path(static_cast<size_t>(state.range(0))));

  for (auto _ : state) {
    nav_msgs::msg::Path out;
    bb->get("path", out);
    benchmark::DoNotOptimize(out);
  }
  state.SetItemsProcessed(state.iterations());
  state.SetBytesProcessed(
    state.iterations() *
    static_cast<int64_t>(
      state.range(0) * static_cast<int64_t>(sizeof(geometry_msgs::msg::PoseStamped)) +
      static_cast<int64_t>(sizeof(std_msgs::msg::Header))));
}

// AFTER: blackboard stores shared_ptr<Path> — get copies only the pointer
static void BM_BlackboardGetPath_SharedPtr(benchmark::State & state)
{
  auto bb = BT::Blackboard::create();
  bb->set(
    "path", std::make_shared<nav_msgs::msg::Path>(
      make_path(static_cast<size_t>(state.range(0)))));

  std::shared_ptr<nav_msgs::msg::Path> out;
  for (auto _ : state) {
    bb->get("path", out);
    benchmark::DoNotOptimize(out);
  }
  state.SetItemsProcessed(state.iterations());
}

BENCHMARK(BM_BlackboardGetPath_Copy)
->Arg(100)
->Arg(1000)
->Arg(10000);

BENCHMARK(BM_BlackboardGetPath_SharedPtr)
->Arg(100)
->Arg(1000)
->Arg(10000);

BENCHMARK_MAIN();
