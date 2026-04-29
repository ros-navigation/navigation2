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
 * The filter mask uses occupancy-grid values. State 0 is the reset state;
 * each non-zero state ID maps via configuration to a list of parameter
 * overrides on configured target nodes.
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

protected:
  /**
   * @brief Subscriber callback for the filter info topic.
   */
  void filterInfoCallback(
    const nav2_msgs::msg::CostmapFilterInfo::ConstSharedPtr & msg);

  /**
   * @brief Subscriber callback for the filter mask topic.
   */
  void maskCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);

  /**
   * @brief Parse the per-state parameter map and nominal_defaults from
   *        YAML overrides.
   */
  void loadStateConfig();

  /**
   * @brief Apply the parameter set associated with the given state.
   *        State 0 restores nominal_defaults; throws on unknown state.
   */
  void applyState(uint8_t new_state);

  /**
   * @brief Restore all overridden parameters to their nominal_defaults
   *        values via async set_parameters.
   */
  void resetToNominal();

  /**
   * @brief Issue an async set_parameters call to the named target node.
   */
  void issueAsyncSetParameters(
    const std::string & target_node,
    const std::vector<rclcpp::Parameter> & params);

  /**
   * @brief Drain completed set_parameters futures non-blockingly. Called
   *        at the start of every process() to guard future lifetimes.
   */
  void drainPendingFutures();

  nav2::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  nav2::Publisher<std_msgs::msg::UInt8>::SharedPtr state_event_pub_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask_;
  std::string global_frame_;

  uint8_t current_state_{0};
  bool state_initialized_{false};

  // One per-state-override or per-nominal-default entry.
  struct StateParamEntry
  {
    std::string target_node;
    rclcpp::Parameter param;
  };
  std::map<uint8_t, std::vector<StateParamEntry>> state_param_map_;
  // Keyed by target_node; values are bare-named Parameters to restore.
  std::map<std::string, std::vector<rclcpp::Parameter>> nominal_defaults_;
  std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr> param_clients_;

  std::vector<std::shared_future<
      std::vector<rcl_interfaces::msg::SetParametersResult>>> pending_futures_;

  std::string state_event_topic_;

  enum class ParamSetFailurePolicy { kWarn, kThrow };
  ParamSetFailurePolicy param_set_failure_policy_{ParamSetFailurePolicy::kThrow};

  bool filter_info_received_{false};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__ZONE_PARAMETER_FILTER_HPP_
