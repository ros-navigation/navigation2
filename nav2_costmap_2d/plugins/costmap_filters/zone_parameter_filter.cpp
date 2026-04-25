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

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_utils.hpp"

namespace nav2_costmap_2d
{

namespace
{
// Separator between target-node-name and parameter-name within a stored
// rclcpp::Parameter's .get_name(). Chosen because ROS 2 parameter names
// allow dots and slashes but not colons, so this is collision-free.
constexpr char kNodeParamSep = ':';
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

  // Per-plugin parameters specific to ZoneParameterFilter.
  state_event_topic_ =
    node->declare_or_get_parameter<std::string>(name_ + "." + "state_event_topic", std::string(""));

  std::string unknown_state_str =
    node->declare_or_get_parameter<std::string>(name_ + "." + "on_unknown_state",
      std::string("warn"));
  if (unknown_state_str == "throw") {
    unknown_state_policy_ = UnknownStatePolicy::kThrow;
  } else {
    unknown_state_policy_ = UnknownStatePolicy::kWarn;
    if (unknown_state_str != "warn") {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: on_unknown_state=%s not recognised; defaulting to 'warn'.",
        unknown_state_str.c_str());
    }
  }

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

  // Filter info subscription — same shape as SpeedFilter / BinaryFilter.
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

  // Optional state-event publisher.
  if (!state_event_topic_.empty()) {
    state_event_pub_ =
      node->create_publisher<std_msgs::msg::UInt8>(joinWithParentNamespace(state_event_topic_));
    state_event_pub_->on_activate();
  }

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

  // ZoneParameterFilter is config-driven — base/multiplier are unused.
  // Mirror KeepoutFilter's check (it requires defaults too).
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

  // For each state_id, expect a sub-namespace `state_<N>.<target_node>.<param_path>: value`
  // in the YAML overrides. We discover them by iterating overrides with the matching prefix.
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
      // override_name is like "<plugin_name>.state_<N>.<target_node>.<param_path>"
      // Strip the prefix to get "<target_node>.<param_path>".
      std::string suffix = override_name.substr(state_prefix.size());
      // Split on the first dot — left of dot = target_node, right = param_path.
      auto dot_pos = suffix.find('.');
      if (dot_pos == std::string::npos) {
        RCLCPP_WARN(
          logger_,
          "ZoneParameterFilter: state_%u entry %s has no dot — expected "
          "<target_node>.<param_path>; skipping.",
          state_id, override_name.c_str());
        continue;
      }
      std::string target_node = suffix.substr(0, dot_pos);
      std::string target_param = suffix.substr(dot_pos + 1);
      // Stored Parameter name is "<target_node>:<target_param>"; the colon
      // separator routes applyState() to the right per-node async client.
      params_for_state.emplace_back(
        target_node + kNodeParamSep + target_param, override_value);
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

  // Load YAML-declared nominal defaults for state-0 reset. Format:
  //   <plugin>.nominal_defaults.<target_node>.<param_path>: <value>
  //
  // Declarative-explicit (rather than auto-captured) because get_parameters
  // and set_parameters use separate underlying services::Client instances,
  // so FIFO ordering of "capture-then-override" cannot be guaranteed at the
  // server. A late get response would capture the overridden value, not the
  // nominal. YAML nominals avoid the race entirely and match Steve Macenski's
  // config-driven preference on #6080.
  const std::string nominal_prefix = name_ + ".nominal_defaults.";
  for (const auto & [override_name, override_value] : overrides) {
    if (override_name.rfind(nominal_prefix, 0) != 0) {
      continue;
    }
    std::string suffix = override_name.substr(nominal_prefix.size());
    auto dot_pos = suffix.find('.');
    if (dot_pos == std::string::npos) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: nominal_defaults entry '%s' has no dot — expected "
        "<target_node>.<param_path>; skipping.",
        override_name.c_str());
      continue;
    }
    const std::string target_node = suffix.substr(0, dot_pos);
    const std::string target_param = suffix.substr(dot_pos + 1);
    const std::string stored_name = target_node + kNodeParamSep + target_param;
    nominal_defaults_[stored_name] = rclcpp::Parameter(target_param, override_value);
  }
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu nominal default(s) loaded for state-0 reset.",
    nominal_defaults_.size());

  // Quality-of-life: warn for any state-N override that has no nominal
  // counterpart. Such params will NOT be restored by state-0 reset; a v1
  // user almost certainly wants symmetric coverage.
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

  // Drain any completed futures from prior set_parameters calls. Must not
  // block — the future-lifetime regression in #3796 review item 5 was
  // exactly about destructing futures before callbacks fire.
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
    // Robot is outside mask range — treat as reset state to be safe.
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
    if (unknown_state_policy_ == UnknownStatePolicy::kThrow) {
      throw std::runtime_error(
              std::string("ZoneParameterFilter: unknown state ") +
              std::to_string(new_state) + " encountered (on_unknown_state=throw)");
    }
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: state %u not in configuration; not changing parameters "
      "(on_unknown_state=warn). Add to state_ids + state_%u map to enable.",
      new_state, new_state);
    return;
  }

  // Group params by target node so each AsyncParametersClient set call
  // batches all params for that node — Steve's #3796 first review point.
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

    // Nominal defaults are loaded declaratively from YAML at config time
    // (see loadStateConfig — `<plugin>.nominal_defaults.<node>.<param>`).
    // No auto-capture here: an async get_parameters round-trip racing
    // against this set_parameters call cannot guarantee ordering at the
    // server, and blocking in process() is forbidden by #3796 review item 2.
    // If the user did not declare a nominal for this param, state-0 reset
    // will skip it (warned at config-load time).
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

  // Lazily build per-node async client.
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

  // Per #3796 review item 2: never wait_for_service in the hot path.
  // Use service_is_ready() as a non-blocking probe; if not ready, log + skip.
  if (!client_it->second->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: parameter service for node '%s' not ready; skipping set.",
      target_node.c_str());
    return;
  }

  // AsyncParametersClient::set_parameters returns std::shared_future directly;
  // no .share() needed (it's already shared).
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
  // Keep nominal_defaults_ + param_clients_ across resets — they're cheap
  // to retain, and re-discovering them every reset would cause extra
  // service round-trips.
}

bool ZoneParameterFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
  return filter_mask_ != nullptr;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ZoneParameterFilter, nav2_costmap_2d::Layer)
