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

#include "nav2_costmap_2d/costmap_filters/zone_parameter_filter.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_utils.hpp"

namespace nav2_costmap_2d
{

namespace
{
// Longest-prefix match against `sorted_nodes` (sorted by length descending),
// so nested-namespace targets resolve unambiguously.
// Returns (target_node, param_path) on hit, or std::nullopt on miss.
std::optional<std::pair<std::string, std::string>>
matchTargetNode(
  const std::string & suffix, const std::vector<std::string> & sorted_nodes)
{
  for (const auto & node : sorted_nodes) {
    const size_t boundary = node.size();
    if (suffix.size() > boundary + 1 &&
      suffix.compare(0, boundary, node) == 0 &&
      suffix[boundary] == '.')
    {
      return std::make_pair(node, suffix.substr(boundary + 1));
    }
  }
  return std::nullopt;
}
}  // namespace

ZoneParameterFilter::ZoneParameterFilter()
: filter_info_sub_(nullptr),
  mask_sub_(nullptr),
  state_event_pub_(nullptr),
  filter_mask_(nullptr),
  global_frame_("")
{
}

void ZoneParameterFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  nav2::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  state_event_topic_ =
    node->declare_or_get_parameter<std::string>(
    name_ + "." + "state_event_topic", std::string("zone_filter_state"));

  std::string param_set_failure_str =
    node->declare_or_get_parameter<std::string>(
    name_ + "." + "on_param_set_failure", std::string("throw"));
  if (param_set_failure_str == "warn") {
    param_set_failure_policy_ = ParamSetFailurePolicy::kWarn;
  } else {
    param_set_failure_policy_ = ParamSetFailurePolicy::kThrow;
    if (param_set_failure_str != "throw") {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: on_param_set_failure=%s not recognised; defaulting to 'throw'.",
        param_set_failure_str.c_str());
    }
  }

  filter_info_topic_ = joinWithParentNamespace(filter_info_topic);
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_,
    std::bind(&ZoneParameterFilter::filterInfoCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS());

  global_frame_ = layered_costmap_->getGlobalFrameID();

  state_event_pub_ =
    node->create_publisher<std_msgs::msg::UInt8>(joinWithParentNamespace(state_event_topic_));
  state_event_pub_->on_activate();

  // Load the per-state parameter map from YAML overrides.
  loadStateConfig();
}

void ZoneParameterFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::ConstSharedPtr & msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  nav2::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: New costmap filter info arrived from %s topic. "
      "Updating old filter info.",
      filter_info_topic_.c_str());
    mask_sub_.reset();
  }

  if (msg->type != ZONE_PARAMETER_FILTER) {
    RCLCPP_ERROR(
      logger_,
        "ZoneParameterFilter: CostmapFilterInfo type is %i, expected %i (ZONE_PARAMETER_FILTER)",
      msg->type, ZONE_PARAMETER_FILTER);
    return;
  }

  // base/multiplier unused (config-driven).
  if (msg->base != BASE_DEFAULT || msg->multiplier != MULTIPLIER_DEFAULT) {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: base=%f and multiplier=%f are unused by this filter "
      "(state mapping is config-driven). Expected defaults (%f, %f).",
      msg->base, msg->multiplier, BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  filter_info_received_ = true;
  mask_topic_ = joinWithParentNamespace(msg->filter_mask_topic);

  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_,
    std::bind(&ZoneParameterFilter::maskCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS(3));
}

void ZoneParameterFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;
}

void ZoneParameterFilter::loadStateConfig()
{
  nav2::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // state_ids: required parameter listing the configured states.
  // Each value is a uint8 in [1, 255] (0 is reserved for reset).
  std::vector<int64_t> state_ids =
    node->declare_or_get_parameter<std::vector<int64_t>>(
    name_ + "." + "state_ids", std::vector<int64_t>{});

  if (state_ids.empty()) {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: state_ids is empty; this filter will only handle "
      "state 0 (reset). Configure state_ids: [1, 2, ...] in YAML.");
    return;
  }

  // target_nodes: explicit list required because ROS 2 parameter names
  // admit dots; without it the parser cannot locate the node-name boundary
  // in a flattened override key like `<plugin>.state_1.<target>.<param.with.dots>`.
  std::vector<std::string> target_nodes_input =
    node->declare_or_get_parameter<std::vector<std::string>>(
    name_ + ".target_nodes", std::vector<std::string>{});

  if (target_nodes_input.empty()) {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: target_nodes is empty; cannot parse state overrides. "
      "Set `target_nodes: [node_a, node_b]` in YAML.");
    return;
  }

  // Length-descending so longest-prefix wins (nested namespaces).
  std::vector<std::string> sorted_target_nodes = target_nodes_input;
  std::sort(
    sorted_target_nodes.begin(), sorted_target_nodes.end(),
    [](const std::string & a, const std::string & b) {return a.size() > b.size();});

  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu target_nodes registered.", sorted_target_nodes.size());

  auto overrides = node->get_node_parameters_interface()->get_parameter_overrides();

  for (int64_t id_i64 : state_ids) {
    if (id_i64 <= 0 || id_i64 > 255) {
      RCLCPP_ERROR(
        logger_,
        "ZoneParameterFilter: state_id %ld is outside valid range [1, 255]; skipping.",
        id_i64);
      continue;
    }
    const uint8_t state_id = static_cast<uint8_t>(id_i64);
    const std::string state_prefix = name_ + ".state_" + std::to_string(state_id) + ".";

    std::vector<StateParamEntry> params_for_state;
    for (const auto & [override_name, override_value] : overrides) {
      if (override_name.rfind(state_prefix, 0) != 0) {
        continue;  // not for this state
      }
      const std::string suffix = override_name.substr(state_prefix.size());
      const auto match = matchTargetNode(suffix, sorted_target_nodes);
      if (!match) {
        RCLCPP_WARN(
          logger_,
          "ZoneParameterFilter: state_%u entry '%s' did not match any registered "
          "target_node. Add the node to `target_nodes` in YAML.",
          state_id, override_name.c_str());
        continue;
      }
      params_for_state.push_back(
        StateParamEntry{match->first, rclcpp::Parameter(match->second, override_value)});
    }

    if (params_for_state.empty()) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: state_id %u declared but no parameters found "
        "under prefix '%s'.",
        state_id, state_prefix.c_str());
    }

    state_param_map_[state_id] = std::move(params_for_state);
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: state_%u → %zu parameter override(s).",
      state_id, state_param_map_[state_id].size());
  }

  // Declarative YAML nominals — auto-capture races on separate underlying
  // services::Client instances for get/set on the target.
  const std::string nominal_prefix = name_ + ".nominal_defaults.";
  size_t nominal_count = 0;
  for (const auto & [override_name, override_value] : overrides) {
    if (override_name.rfind(nominal_prefix, 0) != 0) {
      continue;
    }
    const std::string suffix = override_name.substr(nominal_prefix.size());
    const auto match = matchTargetNode(suffix, sorted_target_nodes);
    if (!match) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: nominal_defaults entry '%s' did not match any "
        "registered target_node. Add the node to `target_nodes` in YAML.",
        override_name.c_str());
      continue;
    }
    nominal_defaults_[match->first].emplace_back(match->second, override_value);
    ++nominal_count;
  }
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu nominal default(s) loaded for state-0 reset.",
    nominal_count);

  // Warn on state-N override without matching nominal (state-0 reset gap).
  auto has_nominal = [this](const StateParamEntry & e) -> bool {
      const auto node_it = nominal_defaults_.find(e.target_node);
      if (node_it == nominal_defaults_.end()) {
        return false;
      }
      const auto & param_name = e.param.get_name();
      for (const auto & nominal : node_it->second) {
        if (nominal.get_name() == param_name) {
          return true;
        }
      }
      return false;
    };
  for (const auto & [state_id, entries] : state_param_map_) {
    for (const auto & entry : entries) {
      if (!has_nominal(entry)) {
        RCLCPP_WARN(
          logger_,
          "ZoneParameterFilter: state_%u sets '%s:%s' but no matching nominal_defaults "
          "entry exists; state-0 reset will NOT restore this parameter.",
          state_id, entry.target_node.c_str(), entry.param.get_name().c_str());
      }
    }
  }

  // Build one AsyncParametersClient per unique target node (init-time;
  // not lazy). Client construction registers locally; the remote service
  // need not be reachable yet — set_parameters failures surface via
  // drainPendingFutures.
  std::set<std::string> all_target_nodes;
  for (const auto & [_state_id, entries] : state_param_map_) {
    for (const auto & e : entries) {
      all_target_nodes.insert(e.target_node);
    }
  }
  for (const auto & [target_node, _params] : nominal_defaults_) {
    all_target_nodes.insert(target_node);
  }
  for (const auto & target_node : all_target_nodes) {
    param_clients_.emplace(
      target_node,
      std::make_shared<rclcpp::AsyncParametersClient>(
        node->get_node_base_interface(),
        node->get_node_topics_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        target_node));
  }
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu AsyncParametersClient(s) built at init.",
    param_clients_.size());
}

void ZoneParameterFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  // Drain completed futures non-blockingly (must not destruct in-flight).
  drainPendingFutures();

  if (!filter_mask_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: Filter mask was not received");
    return;
  }

  // Transform pose into mask frame.
  geometry_msgs::msg::Pose mask_pose;
  if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
    return;
  }

  // Sample mask at robot pose.
  unsigned int mask_robot_i, mask_robot_j;
  if (!nav2_util::worldToMap(
      filter_mask_, mask_pose.position.x, mask_pose.position.y,
      mask_robot_i, mask_robot_j))
  {
    if (state_initialized_ && current_state_ != 0) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: Robot outside filter mask; resetting to nominal defaults.");
      applyState(0);
      current_state_ = 0;
    }
    return;
  }

  const int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (mask_data < 0) {
    // OCC_GRID_UNKNOWN (-1) — leave state alone, log throttled warning.
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: Filter mask cell [%u, %u] is unknown; not changing state.",
      mask_robot_i, mask_robot_j);
    return;
  }

  const uint8_t new_state = static_cast<uint8_t>(mask_data);

  if (state_initialized_ && new_state == current_state_) {
    return;  // No change.
  }

  applyState(new_state);

  if (state_event_pub_) {
    auto event_msg = std::make_unique<std_msgs::msg::UInt8>();
    event_msg->data = new_state;
    state_event_pub_->publish(std::move(event_msg));
  }

  current_state_ = new_state;
  state_initialized_ = true;
}

void ZoneParameterFilter::applyState(uint8_t new_state)
{
  if (new_state == 0) {
    resetToNominal();
    RCLCPP_INFO(logger_, "ZoneParameterFilter: Entered state 0 (reset to nominal).");
    return;
  }

  auto it = state_param_map_.find(new_state);
  if (it == state_param_map_.end()) {
    // Configuration / data-integrity fault: the mask emitted a state value
    // not in our configured map. Throw so the lifecycle layer surfaces the
    // fault — silently continuing with stale parameters is strictly worse
    // than a fail-safe stack-stop for any safety-relevant downstream.
    throw std::runtime_error(
            std::string("ZoneParameterFilter: unknown state ") +
            std::to_string(new_state) +
            " encountered; add to state_ids + state_" +
            std::to_string(new_state) + " map.");
  }

  // Batch per target node (one set_parameters call per node).
  std::map<std::string, std::vector<rclcpp::Parameter>> per_node_params;
  for (const auto & entry : it->second) {
    per_node_params[entry.target_node].push_back(entry.param);
  }

  for (const auto & [target_node, params] : per_node_params) {
    issueAsyncSetParameters(target_node, params);
  }

  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Entered state %u (applied %zu parameter(s) across %zu node(s)).",
    new_state, it->second.size(), per_node_params.size());
}

void ZoneParameterFilter::resetToNominal()
{
  for (const auto & [target_node, params] : nominal_defaults_) {
    issueAsyncSetParameters(target_node, params);
  }
}

void ZoneParameterFilter::issueAsyncSetParameters(
  const std::string & target_node,
  const std::vector<rclcpp::Parameter> & params)
{
  // Clients are built at init in loadStateConfig(); a missing entry here
  // indicates a state references a target_node that wasn't enumerated at
  // config-load (programmer error). Failed set_parameters surfaces via
  // drainPendingFutures + param_set_failure_policy_, not by skip-here.
  auto client_it = param_clients_.find(target_node);
  if (client_it == param_clients_.end()) {
    RCLCPP_ERROR(
      logger_,
      "ZoneParameterFilter: no client for target_node '%s' "
      "(should have been built at config-load).",
      target_node.c_str());
    return;
  }

  pending_futures_.push_back(client_it->second->set_parameters(params));
}

void ZoneParameterFilter::drainPendingFutures()
{
  auto it = pending_futures_.begin();
  while (it != pending_futures_.end()) {
    if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto results = it->get();
        for (const auto & r : results) {
          if (!r.successful) {
            if (param_set_failure_policy_ == ParamSetFailurePolicy::kThrow) {
              throw std::runtime_error(
                      "ZoneParameterFilter: set_parameters failed: " + r.reason);
            }
            RCLCPP_WARN(
              logger_,
              "ZoneParameterFilter: set_parameters returned unsuccessful: %s",
              r.reason.c_str());
          }
        }
      } catch (const std::exception & ex) {
        if (param_set_failure_policy_ == ParamSetFailurePolicy::kThrow) {
          throw;
        }
        RCLCPP_WARN(
          logger_, "ZoneParameterFilter: future.get() threw: %s", ex.what());
      }
      it = pending_futures_.erase(it);
    } else {
      ++it;
    }
  }
}

void ZoneParameterFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (state_event_pub_) {
    state_event_pub_->on_deactivate();
    state_event_pub_.reset();
  }

  filter_mask_.reset();
  filter_info_received_ = false;
  state_initialized_ = false;
  current_state_ = 0;
  // Retain nominal_defaults_ + param_clients_ across resets.
}

bool ZoneParameterFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
  return filter_mask_ != nullptr;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ZoneParameterFilter, nav2_costmap_2d::Layer)
