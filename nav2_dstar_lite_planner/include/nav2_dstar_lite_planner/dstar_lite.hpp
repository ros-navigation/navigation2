// Copyright (c) 2024 Nav2 Contributors
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

#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_HPP_

#include <cmath>
#include <chrono>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_dstar_lite_planner/parameter_handler.hpp"

namespace nav2_dstar_lite_planner
{

static constexpr double INF_COST = std::numeric_limits<double>::max();

static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;

struct CellIndex
{
  int x;
  int y;

  bool operator==(const CellIndex & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex & idx) const
  {
    return static_cast<std::size_t>(idx.x) ^
           (static_cast<std::size_t>(idx.y) << 16);
  }
};

struct Key
{
  double k1;
  double k2;

  bool operator<(const Key & other) const
  {
    if (k1 != other.k1) {return k1 < other.k1;}
    return k2 < other.k2;
  }

  bool operator>(const Key & other) const
  {
    if (k1 != other.k1) {return k1 > other.k1;}
    return k2 > other.k2;
  }

  bool operator==(const Key & other) const
  {
    return k1 == other.k1 && k2 == other.k2;
  }
};

struct DStarLiteState
{
  double g{INF_COST};
  double rhs{INF_COST};
  Key key{};
  bool in_queue{false};
};

struct WorldCoord
{
  double x;
  double y;
};

struct PriorityQueueEntry
{
  CellIndex index;
  Key key;
};

struct PriorityQueueCompare
{
  bool operator()(const PriorityQueueEntry & a, const PriorityQueueEntry & b)
  {
    return a.key > b.key;
  }
};

class DStarLite
{
public:
  CellIndex src_{};
  CellIndex dst_{};
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  int size_x_{0};
  int size_y_{0};

  int nodes_opened{0};
  int replan_count_{0};

  explicit DStarLite(Parameters * params);
  ~DStarLite() = default;

  bool generatePath(
    std::vector<WorldCoord> & path,
    std::function<bool()> cancel_checker);

  void setStartAndGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  bool isUnsafeToPlan() const;

  void forceFullReplan()
  {
    needs_full_replan_ = true;
  }

  void clearStart();

  bool isFirstRun() const
  {
    return first_run_;
  }

  double lastPathCost() const
  {
    return last_path_cost_;
  }

protected:
  Parameters * params_;

  std::vector<DStarLiteState> state_pool_;
  std::unordered_map<CellIndex, int, CellIndexHash> state_lookup_;
  std::priority_queue<
    PriorityQueueEntry,
    std::vector<PriorityQueueEntry>,
    PriorityQueueCompare
  > queue_;
  int next_state_id_{0};

  std::vector<unsigned char> prev_costmap_;
  bool first_run_{true};
  bool needs_full_replan_{false};

  double last_path_cost_{INF_COST};

  static constexpr int dx_[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static constexpr int dy_[8] = {1, -1, 0, 0, 1, 1, -1, -1};

  void initialize();

  void computeShortestPath(std::function<bool()> cancel_checker);

  void updateVertex(const CellIndex & s);

  Key calculateKey(const CellIndex & s);

  double getHeuristic(const CellIndex & s) const;

  double getEdgeCost(const CellIndex & from, const CellIndex & to) const;

  double getTraversalCost(const CellIndex & s) const;

  bool isSafe(const CellIndex & s) const;

  bool withinBounds(const CellIndex & s) const;

  std::vector<CellIndex> getSuccessors(const CellIndex & s) const;

  std::vector<CellIndex> getPredecessors(const CellIndex & s) const;

  double extractPath(std::vector<WorldCoord> & path);

  std::vector<CellIndex> getChangedCells();

  void snapshotCostmap();

  DStarLiteState & getOrCreateState(const CellIndex & s);

  void resetState();

  void clearQueue();
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_HPP_
