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

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();
  state_event_topic_ =
    node->declare_or_get_parameter<std::string>(
    name_ + "." + "state_event_topic", std::string("zone_filter_state"));
  filter_info_topic_ = joinWithParentNamespace(filter_info_topic);
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());

  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_,
    std::bind(&ZoneParameterFilter::filterInfoCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS());

  state_event_pub_ =
    node->create_publisher<std_msgs::msg::UInt8>(joinWithParentNamespace(state_event_topic_));
  state_event_pub_->on_activate();

  loadStateConfig();
}

void ZoneParameterFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::ConstSharedPtr & msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  auto node = node_.lock();
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
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Obtain the node, parameter, and value for state entries
  auto read_entry =
    [&](const std::string & prefix) -> std::optional<StateParamEntry> {
      const std::string target_node =
        node->declare_or_get_parameter<std::string>(prefix + ".node", std::string(""));
      const std::string param_name =
        node->declare_or_get_parameter<std::string>(prefix + ".parameter", std::string(""));
      if (target_node.empty() || param_name.empty()) {
        RCLCPP_ERROR(
          logger_,
          "ZoneParameterFilter: '%s' must declare non-empty 'node' and 'parameter'.",
          prefix.c_str());
        return std::nullopt;
      }
      const std::string value_key = prefix + ".value";
      if (!node->has_parameter(value_key)) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;
        node->declare_parameter(value_key, rclcpp::ParameterValue{}, descriptor);
      }
      const rclcpp::Parameter value_param = node->get_parameter(value_key);
      if (value_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        RCLCPP_ERROR(logger_, "ZoneParameterFilter: '%s' is not set.", value_key.c_str());
        return std::nullopt;
      }
      return StateParamEntry{
      target_node, rclcpp::Parameter(param_name, value_param.get_parameter_value())};
    };

  const std::vector<std::string> state_names =
    node->declare_or_get_parameter<std::vector<std::string>>(
    name_ + ".states", std::vector<std::string>{});

  if (state_names.empty()) {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: 'states' is empty; this filter will only handle "
      "state 0 (reset). Configure `states: [name_a, ...]` in YAML.");
  }

  for (const auto & state_name : state_names) {
    const std::string state_prefix = name_ + "." + state_name;

    const int64_t id_i64 =
      node->declare_or_get_parameter<int64_t>(state_prefix + ".id", 0);
    if (id_i64 <= 0 || id_i64 > 255) {
      RCLCPP_ERROR(
        logger_,
        "ZoneParameterFilter: state '%s' has id %ld outside the valid range "
        "[1, 255] (0 is reserved for reset); skipping.",
        state_name.c_str(), id_i64);
      continue;
    }
    const uint8_t state_id = static_cast<uint8_t>(id_i64);

    const std::vector<std::string> setpoint_names =
      node->declare_or_get_parameter<std::vector<std::string>>(
      state_prefix + ".setpoints", std::vector<std::string>{});

    std::vector<StateParamEntry> params_for_state;
    for (const auto & setpoint_name : setpoint_names) {
      if (auto entry = read_entry(state_prefix + "." + setpoint_name)) {
        params_for_state.push_back(std::move(*entry));
      }
    }

    if (params_for_state.empty()) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: state '%s' (id %u) declares no valid setpoints.",
        state_name.c_str(), state_id);
    }

    state_param_map_[state_id] = std::move(params_for_state);
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: state '%s' (id %u) -> %zu setpoint(s).",
      state_name.c_str(), state_id, state_param_map_[state_id].size());
  }

  // `nominal_defaults`: the baseline values restored on the state-0 reset.
  const std::vector<std::string> nominal_names =
    node->declare_or_get_parameter<std::vector<std::string>>(
    name_ + ".nominal_defaults", std::vector<std::string>{});
  for (const auto & nominal_name : nominal_names) {
    if (auto entry = read_entry(name_ + ".nominal_defaults." + nominal_name)) {
      nominal_defaults_[entry->target_node].push_back(entry->param);
    }
  }
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu nominal default(s) loaded for state-0 reset.",
    nominal_names.size());

  // Warn on a state override with no matching nominal_default: the state-0
  // reset would not be able to restore that parameter.
  auto has_nominal = [this](const StateParamEntry & e) -> bool {
      const auto node_it = nominal_defaults_.find(e.target_node);
      if (node_it == nominal_defaults_.end()) {
        return false;
      }
      for (const auto & nominal : node_it->second) {
        if (nominal.get_name() == e.param.get_name()) {
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
          "ZoneParameterFilter: state id %u sets '%s' on '%s' but no matching "
          "nominal_defaults entry exists; state-0 reset will NOT restore it.",
          state_id, entry.param.get_name().c_str(), entry.target_node.c_str());
      }
    }
  }

  // Per-node client construction
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

  checkPendingParameterUpdates();

  if (!filter_mask_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose mask_pose;
  if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
    return;
  }

  unsigned int mask_robot_i, mask_robot_j;
  if (!nav2_util::worldToMap(
      filter_mask_, mask_pose.position.x, mask_pose.position.y,
      mask_robot_i, mask_robot_j))
  {
    if (!state_initialized_ || current_state_ != 0) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: Robot outside filter mask; applying nominal defaults.");
      if (pending_sets_.empty()) {
        reapply_after_drain_ = false;
      }
      enterState(0);
    } else {
      reapplyAfterDrainIfDue();
    }
    return;
  }

  const int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (mask_data < 0) {
    // mask_data < 0 is OCC_GRID_UNKNOWN; don't change state on an unknown cell.
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: Filter mask cell [%u, %u] is unknown; not changing state.",
      mask_robot_i, mask_robot_j);
    reapplyAfterDrainIfDue();
    return;
  }

  const uint8_t new_state = static_cast<uint8_t>(mask_data);

  if (state_initialized_ && new_state == current_state_) {
    reapplyAfterDrainIfDue();
    return;
  }

  // A transition supersedes the pending re-apply only once its sets have
  // drained; while they are in flight one can still land last.
  if (pending_sets_.empty()) {
    reapply_after_drain_ = false;
  }
  enterState(new_state);
}

void ZoneParameterFilter::enterState(uint8_t new_state)
{
  applyState(new_state);

  // Every route into a state is announced, including both routes to state 0.
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
    throw std::runtime_error(
            std::string("ZoneParameterFilter: unknown state ") +
            std::to_string(new_state) +
            " encountered; declare a state with id " +
            std::to_string(new_state) + " under the filter's `states` list.");
  }

  std::set<std::pair<std::string, std::string>> m_keys;
  for (const auto & entry : it->second) {
    m_keys.emplace(entry.target_node, entry.param.get_name());
  }

  // Reset params touched by the previous state but not the destination.
  std::map<std::string, std::vector<rclcpp::Parameter>> reset_per_node;
  if (state_initialized_ && current_state_ != 0) {
    auto prev_it = state_param_map_.find(current_state_);
    if (prev_it != state_param_map_.end()) {
      for (const auto & entry : prev_it->second) {
        const auto key = std::make_pair(entry.target_node, entry.param.get_name());
        if (m_keys.count(key) > 0) {
          continue;  // M will set this param; reset is wasted work.
        }
        const auto node_it = nominal_defaults_.find(entry.target_node);
        if (node_it == nominal_defaults_.end()) {
          // No nominal_defaults were declared for this node, so this param has
          // nothing to reset to; it keeps state N's value. Warned at config-load.
          continue;
        }
        for (const auto & nominal : node_it->second) {
          if (nominal.get_name() == entry.param.get_name()) {
            reset_per_node[entry.target_node].push_back(nominal);
            break;
          }
        }
      }
    } else {
      RCLCPP_ERROR(
        logger_,
        "ZoneParameterFilter: current state %u is not in state_param_map_ "
        "(should have been recorded when the state was applied).",
        current_state_);
    }
  }

  // Batch per target node (one set_parameters call per node).
  std::map<std::string, std::vector<rclcpp::Parameter>> per_node_params;
  for (const auto & entry : it->second) {
    per_node_params[entry.target_node].push_back(entry.param);
  }

  // Submit the N-only resets before M's overrides.
  size_t reset_count = 0;
  for (const auto & [target_node, params] : reset_per_node) {
    issueAsyncSetParameters(target_node, params);
    reset_count += params.size();
  }

  for (const auto & [target_node, params] : per_node_params) {
    issueAsyncSetParameters(target_node, params);
  }

  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Entered state %u (reset %zu N-only parameter(s); "
    "applied %zu parameter(s) across %zu node(s)).",
    new_state, reset_count, it->second.size(), per_node_params.size());
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
  auto client_it = param_clients_.find(target_node);
  if (client_it == param_clients_.end()) {
    RCLCPP_ERROR(
      logger_,
      "ZoneParameterFilter: no client for target_node '%s' "
      "(should have been built at config-load).",
      target_node.c_str());
    return;
  }

  if (pending_sets_.size() >= kMaxPendingSets) {
    // Only reachable when a target has stopped answering. Client and future go
    // together, so nothing is left holding a broken promise.
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: %zu parameter set(s) still in flight; dropping the "
      "oldest unanswered one. Is a target node not responding?",
      pending_sets_.size());
    pending_sets_.erase(pending_sets_.begin());
  }
  pending_sets_.push_back(
    PendingSet{client_it->second, client_it->second->set_parameters(params)});
}

void ZoneParameterFilter::checkPendingParameterUpdates()
{
  // Failures throw rather than being logged and ignored.
  // wait_for(0s) polls without blocking the costmap update loop
  auto it = pending_sets_.begin();
  while (it != pending_sets_.end()) {
    if (it->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      ++it;
      continue;
    }

    // Copy the shared state out before erasing: the copy keeps the value that
    // get() returns a reference to alive for this iteration, and erasing first
    // means a service-side exception rethrown by get() surfaces exactly once.
    const auto ready_future = it->future;
    it = pending_sets_.erase(it);

    // get() returns a const reference; bind it rather than copy the vector.
    const auto & results = ready_future.get();
    for (const auto & r : results) {
      if (!r.successful) {
        throw std::runtime_error(
                "ZoneParameterFilter: set_parameters failed: " + r.reason);
      }
    }
  }
}

void ZoneParameterFilter::reapplyAfterDrainIfDue()
{
  if (!reapply_after_drain_ || !pending_sets_.empty()) {
    return;
  }
  reapply_after_drain_ = false;
  const bool state_still_configured =
    current_state_ == 0 || state_param_map_.count(current_state_) > 0;
  if (state_initialized_ && state_still_configured) {
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: re-applying state %u; the sets issued before the "
      "reload have drained.",
      current_state_);
    applyState(current_state_);
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

  // Sampled before the restore below issues its own sets, which would otherwise
  // make this true on every reset.
  const bool sets_in_flight_before_restore = !pending_sets_.empty();

  // Restore nominal defaults before releasing the configuration that defines them.
  if (state_initialized_ && current_state_ != 0) {
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: leaving state %u; restoring nominal defaults.",
      current_state_);
    applyState(0);
  }

  filter_mask_.reset();
  filter_info_received_ = false;
  state_initialized_ = false;
  current_state_ = 0;

  // Everything derived from the declared configuration goes: loadStateConfig()
  // appends to what it finds, and is re-run by `clear_entirely_<costmap>` and
  // by every deactivate/activate cycle, both of which reach here first.
  state_param_map_.clear();
  nominal_defaults_.clear();
  param_clients_.clear();

  // pending_sets_ is deliberately NOT cleared. Each one still owns the client
  // that issued it, so dropping the map above cancels nothing and breaks
  // nothing: the results stay readable and a failed set still surfaces.
  if (!pending_sets_.empty()) {
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: %zu parameter set(s) still in flight across the "
      "reload; their results will still be checked.",
      pending_sets_.size());
  }
  reapply_after_drain_ = sets_in_flight_before_restore;
}

bool ZoneParameterFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
  return filter_mask_ != nullptr;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ZoneParameterFilter, nav2_costmap_2d::Layer)
