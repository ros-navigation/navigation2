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
  binary_state_pub_(nullptr), filter_mask_(nullptr), mask_frame_(""), global_frame_(""),
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

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    logger_,
    "BinaryFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&BinaryFilter::filterInfoCallback, this, std::placeholders::_1));

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
  mask_frame_ = msg->header.frame_id;
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
  if (!transformPose(global_frame_, pose, mask_frame_, mask_pose)) {
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
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::BinaryFilter, nav2_costmap_2d::Layer)
