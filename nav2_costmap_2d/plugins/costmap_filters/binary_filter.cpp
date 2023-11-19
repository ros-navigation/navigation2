/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022 Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#include "nav2_costmap_2d/costmap_filters/binary_filter.hpp"

#include <cmath>
#include <utility>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_costmap_2d
{

BinaryFilter::BinaryFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr),
  binary_state_pub_(nullptr), filter_mask_(nullptr), global_frame_(""),
  default_state_(false), binary_state_(default_state_)
{
}

void BinaryFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Declare parameters specific to BinaryFilter only
  std::string binary_state_topic;
  declareParameter("default_state", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "default_state", default_state_);
  declareParameter("binary_state_topic", rclcpp::ParameterValue("binary_state"));
  node->get_parameter(name_ + "." + "binary_state_topic", binary_state_topic);
  declareParameter("flip_threshold", rclcpp::ParameterValue(50.0));
  node->get_parameter(name_ + "." + "flip_threshold", flip_threshold_);
  declareParameter("binary_parameters", rclcpp::ParameterValue(std::vector<std::string>()));
  // List of binary params to be changed
  std::vector<std::string> binary_parameters;
  node->get_parameter(name_ + "." + "binary_parameters", binary_parameters);
  declareParameter("change_parameter_timeout", rclcpp::ParameterValue(10));
  node->get_parameter(name_ + "." + "change_parameter_timeout", change_parameter_timeout_);

  for (std::string param : binary_parameters) {
    BinaryParameter param_struct;

    declareParameter(param + "." + "node_name", rclcpp::ParameterValue(""));
    node->get_parameter(name_ + "." + param + "." + "node_name", param_struct.node_name);

    declareParameter(param + "." + "param_name", rclcpp::ParameterValue(""));
    node->get_parameter(name_ + "." + param + "." + "param_name", param_struct.param_name);

    // Take default value from parameter server if not specified
    declareParameter(param + "." + "default_state", rclcpp::ParameterValue(default_state_));
    node->get_parameter(name_ + "." + param + "." + "default_state", param_struct.default_state);

    binary_parameters_info_.push_back(param_struct);
  }

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    logger_,
    "BinaryFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&BinaryFilter::filterInfoCallback, this, std::placeholders::_1));

  // Create clients for changing parameters
  for (auto param : binary_parameters_info_) {
    RCLCPP_DEBUG(
      logger_,
      "BinaryFilter: Creating client for changing parameter \"%s\" of node \"%s\"...",
      param.param_name.c_str(), param.node_name.c_str());

    auto change_parameters_client = node->create_client<rcl_interfaces::srv::SetParameters>(
      "/" + param.node_name + "/set_parameters");
    change_parameters_clients_.push_back(change_parameters_client);
  }

  // Get global frame required for binary state publisher
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Create new binary state publisher
  binary_state_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    binary_state_topic, rclcpp::QoS(10));
  binary_state_pub_->on_activate();

  // Reset parameters
  base_ = BASE_DEFAULT;
  multiplier_ = MULTIPLIER_DEFAULT;

  // Initialize state as "false" by-default
  changeState(default_state_);
}

void BinaryFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "BinaryFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  if (msg->type != BINARY_FILTER) {
    RCLCPP_ERROR(logger_, "BinaryFilter: Mode %i is not supported", msg->type);
    return;
  }

  // Set base_ and multiplier_
  base_ = msg->base;
  multiplier_ = msg->multiplier;
  // Set topic name to receive filter mask from
  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  RCLCPP_INFO(
    logger_,
    "BinaryFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&BinaryFilter::maskCallback, this, std::placeholders::_1));
}

void BinaryFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "BinaryFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;
}

void BinaryFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose2D & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "BinaryFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose2D mask_pose;  // robot coordinates in mask frame

  // Transforming robot pose from current layer frame to mask frame
  if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
    return;
  }

  // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
  unsigned int mask_robot_i, mask_robot_j;
  if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
    // Robot went out of mask range. Set "false" state by-default
    RCLCPP_WARN(
      logger_,
      "BinaryFilter: Robot is outside of filter mask. Resetting binary state to default.");
    changeState(default_state_);
    return;
  }

  // Getting filter_mask data from cell where the robot placed
  int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (mask_data == nav2_util::OCC_GRID_UNKNOWN) {
    // Corresponding filter mask cell is unknown.
    // Warn and do nothing.
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "BinaryFilter: Filter mask [%i, %i] data is unknown. Do nothing.",
      mask_robot_i, mask_robot_j);
    return;
  }
  // Check and flip binary state, if necessary
  if (base_ + mask_data * multiplier_ > flip_threshold_) {
    if (binary_state_ == default_state_) {
      changeState(!default_state_);
    }
  } else {
    if (binary_state_ != default_state_) {
      changeState(default_state_);
    }
  }
}

void BinaryFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  RCLCPP_INFO(logger_, "BinaryFilter: Resetting the filter to default state");
  changeState(default_state_);

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (binary_state_pub_) {
    binary_state_pub_->on_deactivate();
    binary_state_pub_.reset();
  }
}

bool BinaryFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

void BinaryFilter::changeState(const bool state)
{
  binary_state_ = state;
  if (state) {
    RCLCPP_INFO(logger_, "BinaryFilter: Switched on");
  } else {
    RCLCPP_INFO(logger_, "BinaryFilter: Switched off");
  }

  // Forming and publishing new BinaryState message
  std::unique_ptr<std_msgs::msg::Bool> msg =
    std::make_unique<std_msgs::msg::Bool>();
  msg->data = state;
  binary_state_pub_->publish(std::move(msg));
  changeParameters(state);
}

void BinaryFilter::changeParameters(const bool state)
{

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  for (size_t param_index = 0; param_index < binary_parameters_info_.size(); ++param_index) {
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>>
    change_parameters_client = change_parameters_clients_.at(param_index);
    if (!change_parameters_client->wait_for_service(
        std::chrono::milliseconds(change_parameter_timeout_)))
    {
      RCLCPP_WARN(
        logger_, "BinaryFilter:  service %s not available. Skipping ...",
        change_parameters_client->get_service_name());
      continue;
    } else {
      RCLCPP_INFO(
        logger_, "BinaryFilter:  service %s available.",
        change_parameters_client->get_service_name());
    }

    // Create a rcl_interfaces::msg::SetParameters client for changing parameters
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    // Set parameters for BinaryFilter
    rcl_interfaces::msg::Parameter bool_param;
    BinaryFilter::BinaryParameter binary_parameter_info = binary_parameters_info_.at(param_index);
    bool_param.name = binary_parameter_info.param_name;
    bool_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;

    const bool param_default_state = binary_parameter_info.default_state;
    if (state == default_state_) {
      // Filter is not flipped
      bool_param.value.bool_value = param_default_state;
    } else {
      // Filter is flipped
      bool_param.value.bool_value = !param_default_state;
    }
    request->parameters.push_back(bool_param);

    RCLCPP_DEBUG(
      logger_, "BinaryFilter: Sending request to set parameter  %s to %s",
      binary_parameter_info.param_name.c_str(),
      bool_param.value.bool_value ? "true" : "false");
    auto future_result = change_parameters_client->async_send_request(request);

    rclcpp::FutureReturnCode return_code = rclcpp::spin_until_future_complete(
      node, future_result, std::chrono::milliseconds(change_parameter_timeout_));
    if (return_code == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future_result.get();
      if (!result->results.at(0).successful) {
        RCLCPP_ERROR(
          logger_, "BinaryFilter: Failed to change parameter %s",
          bool_param.name.c_str());
        // TODO (@enricosutera) replace this once we figure out what to do
        // throw std::runtime_error("BinaryFilter: Can't update binary filter parameter!");
      } else {
        RCLCPP_DEBUG(
          logger_, "BinaryFilter: Successfully changed parameter to %s",
          bool_param.value.bool_value ? "true" : "false");
      }  
    } else if (return_code == rclcpp::FutureReturnCode::INTERRUPTED) {
      throw std::runtime_error("BinaryFilter: Interruped while spinning for parameter update!");
    } else {
      throw std::runtime_error("BinaryFilter: Spinning for parameter update went wrong :( !");
    }
  }
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BinaryFilter, nav2_costmap_2d::Layer)
