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
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_utils.hpp"

namespace nav2_costmap_2d
{

namespace
{
// Colon — collision-free vs ROS 2 param-name grammar (no colons allowed).
constexpr char kNodeParamSep = ':';

// Longest-prefix match against `sorted_nodes` (sorted by length descending),
// so nested-namespace targets resolve unambiguously.
// Returns (target_node, param_path) on hit, or std::nullopt on miss.
std::optional<std::pair<std::string, std::string>>
matchTargetNode(
  const std::string & suffix, const std::vector<std::string> & sorted_nodes)
{
  for (const auto & node : sorted_nodes) {
    if (suffix.size() > node.size() + 1 &&
      suffix.compare(0, node.size(), node) == 0 &&
      suffix[node.size()] == '.')
    {
      return std::make_pair(node, suffix.substr(node.size() + 1));
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
    name_ + "." + "on_param_set_failure", std::string("warn"));
  if (param_set_failure_str == "throw") {
    param_set_failure_policy_ = ParamSetFailurePolicy::kThrow;
  } else {
    param_set_failure_policy_ = ParamSetFailurePolicy::kWarn;
    if (param_set_failure_str != "warn") {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: on_param_set_failure=%s not recognised; defaulting to 'warn'.",
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

    std::vector<rclcpp::Parameter> params_for_state;
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
      params_for_state.emplace_back(
        match->first + kNodeParamSep + match->second, override_value);
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
    const std::string stored_name = match->first + kNodeParamSep + match->second;
    nominal_defaults_[stored_name] = rclcpp::Parameter(match->second, override_value);
  }
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu nominal default(s) loaded for state-0 reset.",
    nominal_defaults_.size());

  // Warn on state-N override without matching nominal (state-0 reset gap).
  for (const auto & [state_id, params] : state_param_map_) {
    for (const auto & p : params) {
      if (nominal_defaults_.find(p.get_name()) == nominal_defaults_.end()) {
        RCLCPP_WARN(
          logger_,
          "ZoneParameterFilter: state_%u sets '%s' but no matching nominal_defaults "
          "entry exists; state-0 reset will NOT restore this parameter.",
          state_id, p.get_name().c_str());
      }
    }
  }
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
  for (const auto & stored : it->second) {
    const std::string stored_name = stored.get_name();
    auto sep_pos = stored_name.find(kNodeParamSep);
    if (sep_pos == std::string::npos) {
      RCLCPP_ERROR(
        logger_,
        "ZoneParameterFilter: stored parameter name '%s' missing node separator; skipping.",
        stored_name.c_str());
      continue;
    }
    const std::string target_node = stored_name.substr(0, sep_pos);
    const std::string target_param = stored_name.substr(sep_pos + 1);

    per_node_params[target_node].emplace_back(target_param, stored.get_parameter_value());
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
  if (nominal_defaults_.empty()) {
    return;  // nothing captured yet; nothing to restore
  }

  std::map<std::string, std::vector<rclcpp::Parameter>> per_node_params;
  for (const auto & [stored_name, nominal_param] : nominal_defaults_) {
    auto sep_pos = stored_name.find(kNodeParamSep);
    if (sep_pos == std::string::npos) {
      continue;
    }
    const std::string target_node = stored_name.substr(0, sep_pos);
    per_node_params[target_node].emplace_back(nominal_param);
  }

  for (const auto & [target_node, params] : per_node_params) {
    issueAsyncSetParameters(target_node, params);
  }
}

void ZoneParameterFilter::issueAsyncSetParameters(
  const std::string & target_node,
  const std::vector<rclcpp::Parameter> & params)
{
  nav2::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    return;
  }

  auto client_it = param_clients_.find(target_node);
  if (client_it == param_clients_.end()) {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      target_node);
    auto inserted = param_clients_.emplace(target_node, std::move(client));
    client_it = inserted.first;
  }

  // Non-blocking probe; never wait_for_service in the hot path.
  if (!client_it->second->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: parameter service for node '%s' not ready; skipping set.",
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
