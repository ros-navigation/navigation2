// Copyright (c) 2026 Komada (aki1770-del)
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

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__ZONE_PARAMETER_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__ZONE_PARAMETER_FILTER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace nav2_costmap_2d
{

/**
 * @class ZoneParameterFilter
 * @brief Costmap filter that applies a configured set of ROS parameters
 *        based on the mask value at the robot's pose.
 *
 * The filter mask uses occupancy-grid values (0..255). Value 0 is reserved
 * as a "reset to nominal defaults" state. Each non-zero value is mapped via
 * this layer's configuration to a list of parameters (any reachable nav2
 * parameter on any node) to set when that state is active.
 *
 * Generalises the pattern of @ref SpeedFilter to N parameters per state.
 * Discussion: https://github.com/ros-navigation/navigation2/issues/6080
 */
class ZoneParameterFilter : public CostmapFilter
{
public:
  ZoneParameterFilter();

  /**
   * @brief Initialise filter, subscribe to filter info / mask, build
   *        per-target-node async parameter clients.
   */
  void initializeFilter(const std::string & filter_info_topic) override;

  /**
   * @brief Sample the mask at the robot pose; if the state changed,
   *        apply the new state's parameter set via async client and,
   *        if configured, publish the state event.
   */
  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose & pose) override;

  /**
   * @brief Reset filter — drop subscriptions, reset publisher, clear
   *        nominal-defaults capture.
   */
  void resetFilter() override;

  /**
   * @brief Whether the filter has received its mask and is operational.
   */
  bool isActive();

private:
  // Subscription wiring — mirrors SpeedFilter / BinaryFilter.
  void filterInfoCallback(
    const nav2_msgs::msg::CostmapFilterInfo::ConstSharedPtr & msg);
  void maskCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);

  // Parameter map loading (declares state_<N>.* params from YAML overrides).
  void loadStateConfig();

  // State-application helpers.
  void applyState(uint8_t new_state);
  void resetToNominal();
  void issueAsyncSetParameters(
    const std::string & target_node,
    const std::vector<rclcpp::Parameter> & params);

  // Drain in-flight set_parameters futures; called at the top of every
  // process() so we never destruct a future before its callback fires
  // (guards against the navigation2#3796 future-lifetime regression).
  void drainPendingFutures();

  // ===== Subscribers / publisher =====
  nav2::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  nav2::Publisher<std_msgs::msg::UInt8>::SharedPtr state_event_pub_;

  // ===== Mask state =====
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask_;
  std::string global_frame_;

  // ===== State tracking =====
  uint8_t current_state_{0};
  bool state_initialized_{false};

  // state -> list of parameters to set when entering this state.
  // Each Parameter's .get_name() is "<target_node>:<param_name>" — colon
  // separates the target node from the parameter on it. Resolved at
  // applyState() time via param_clients_.
  std::map<uint8_t, std::vector<rclcpp::Parameter>> state_param_map_;

  // Captured nominal defaults — keyed by "<target_node>:<param_name>".
  // Lazy-populated the first time we override each parameter; absence
  // from this map means "not yet captured" (no empty-string sentinel
  // per navigation2#3796 fix 4).
  std::map<std::string, rclcpp::Parameter> nominal_defaults_;

  // Per-target-node async parameter clients (one per remote node we
  // ever set parameters on). Built lazily in issueAsyncSetParameters.
  // Steve's preference per #3796 first review: AsyncParametersClient,
  // not per-call /set_parameter services.
  std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr> param_clients_;

  // In-flight set_parameters futures, drained each process() call.
  std::vector<std::shared_future<
      std::vector<rcl_interfaces::msg::SetParametersResult>>> pending_futures_;

  // ===== Configuration =====
  std::string state_event_topic_;

  enum class UnknownStatePolicy { kWarn, kThrow };
  UnknownStatePolicy unknown_state_policy_{UnknownStatePolicy::kWarn};

  enum class ParamSetFailurePolicy { kWarn, kThrow };
  ParamSetFailurePolicy param_set_failure_policy_{ParamSetFailurePolicy::kWarn};

  bool filter_info_received_{false};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__ZONE_PARAMETER_FILTER_HPP_
