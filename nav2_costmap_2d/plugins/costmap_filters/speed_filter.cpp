/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
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

#include "nav2_costmap_2d/costmap_filters/speed_filter.hpp"

#include <cmath>
#include <utility>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_utils.hpp"

namespace nav2_costmap_2d
{

SpeedFilter::SpeedFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr),
  speed_limit_pub_(nullptr), filter_mask_(nullptr), global_frame_(""),
  speed_limit_(NO_SPEED_LIMIT), speed_limit_prev_(NO_SPEED_LIMIT),
  held_lookahead_dist_(0.0), cached_lookahead_start_idx_(0)
{
}

void SpeedFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  nav2::LifecycleNode::SharedPtr node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Declare "speed_limit_topic" parameter specific to SpeedFilter only
  std::string speed_limit_topic = node->declare_or_get_parameter(name_ + "." + "speed_limit_topic",
    std::string("speed_limit"));
  speed_limit_topic = joinWithParentNamespace(speed_limit_topic);

  // Path lookahead parameters
  enable_path_lookahead_ = node->declare_or_get_parameter(
    name_ + "." + "enable_path_lookahead", false);
  max_decel_ = node->declare_or_get_parameter(
    name_ + "." + "max_decel", -0.5);
  min_lookahead_ = node->declare_or_get_parameter(
    name_ + "." + "min_lookahead", 0.3);
  max_lookahead_ = node->declare_or_get_parameter(
    name_ + "." + "max_lookahead", 5.0);
  std::string path_topic = node->declare_or_get_parameter(
    name_ + "." + "path_topic", std::string("plan"));
  std::string odom_topic = node->declare_or_get_parameter(
    name_ + "." + "odom_topic", std::string("odom"));

  // Check params
  if (enable_path_lookahead_) {
    if (max_decel_ >= 0.0) {
      RCLCPP_WARN(
        logger_,
        "SpeedFilter: max_decel should be negative,"
        "lookahead distance will be set to max_lookahead instead");
    }
    if (min_lookahead_ < 0.0) {
      RCLCPP_WARN(
        logger_,
        "SpeedFilter: min_lookahead = %f is negative,"
        "clamping to 0.0m", min_lookahead_);
      min_lookahead_ = 0.0;
    }
    if (max_lookahead_ < min_lookahead_) {
      RCLCPP_WARN(
        logger_,
        "SpeedFilter: max_lookahead = %f is less than min_lookahead = %f,"
        "clamping to min_lookahead.",
        max_lookahead_, min_lookahead_);
      max_lookahead_ = min_lookahead_;
    }
  }

  filter_info_topic_ = joinWithParentNamespace(filter_info_topic);
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_,
    std::bind(&SpeedFilter::filterInfoCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS());

  // Get global frame required for speed limit publisher
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Create new speed limit publisher
  speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic);
  speed_limit_pub_->on_activate();

  // Path subscriptions and odom smoother if lookahead enabled
  if (enable_path_lookahead_) {
    std::string resolved_path_topic = joinWithParentNamespace(path_topic);
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Path lookahead enabled. Subscribing to \"%s\" topic for path...",
      resolved_path_topic.c_str());
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      resolved_path_topic,
      std::bind(&SpeedFilter::pathCallback, this, std::placeholders::_1));

    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(
      node, 0.3, odom_topic);

    lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      name_ + "/lookahead_point");
    lookahead_pub_->on_activate();
  }

  // Reset speed conversion states
  base_ = BASE_DEFAULT;
  multiplier_ = MULTIPLIER_DEFAULT;
  percentage_ = false;
}

void SpeedFilter::filterInfoCallback(
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
      "SpeedFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  // Set base_/multiplier_ or use speed limit in % of maximum speed
  base_ = msg->base;
  multiplier_ = msg->multiplier;
  if (msg->type == SPEED_FILTER_PERCENT) {
    // Using speed limit in % of maximum speed
    percentage_ = true;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using expressed in a percent from maximum speed"
      "speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else if (msg->type == SPEED_FILTER_ABSOLUTE) {
    // Using speed limit in m/s
    percentage_ = false;
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Using absolute speed_limit = %f + filter_mask_data * %f",
      base_, multiplier_);
  } else {
    RCLCPP_ERROR(logger_, "SpeedFilter: Mode is not supported");
    return;
  }

  mask_topic_ = joinWithParentNamespace(msg->filter_mask_topic);

  // Setting new filter mask subscriber
  RCLCPP_INFO(
    logger_,
    "SpeedFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_,
    std::bind(&SpeedFilter::maskCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS(3));
}

void SpeedFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "SpeedFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "SpeedFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;
}

void SpeedFilter::pathCallback(
  const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  current_path_ = std::make_shared<nav_msgs::msg::Path>();

  // Transform path if not in the global frame
  if(msg->header.frame_id != global_frame_) {
    auto transformed_path = std::make_shared<nav_msgs::msg::Path>();
    if(nav2_util::transformPathInTargetFrame(*msg, *transformed_path, *tf_, global_frame_)) {
      current_path_ = transformed_path;
    } else {
      RCLCPP_ERROR(logger_,
          "SpeedFilter: Failed to transform path to global frame, skipping path lookahead");
    }
  } else {
    current_path_ = msg;
  }

  // Reset cached start index when new path is received
  cached_lookahead_start_idx_ = 0;
}

bool SpeedFilter::getSpeedLimitAtPose(
  const geometry_msgs::msg::Pose & pose,
  double & speed_limit)
{
  geometry_msgs::msg::Pose mask_pose;  // robot coordinates in mask frame

  // Transforming robot pose from current layer frame to mask frame
  if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
    return false;
  }

  // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
  unsigned int mask_robot_i, mask_robot_j;
  if (!nav2_util::worldToMap(
      filter_mask_, mask_pose.position.x, mask_pose.position.y,
      mask_robot_i, mask_robot_j))
  {
    return false;
  }

  // Getting filter_mask data from cell where the robot placed and
  // calculating speed limit value
  int8_t speed_mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (speed_mask_data == SPEED_MASK_NO_LIMIT) {
    // Corresponding filter mask cell is free.
    // Setting no speed limit there.
    speed_limit = NO_SPEED_LIMIT;
  } else if (speed_mask_data == SPEED_MASK_UNKNOWN) {
    // Corresponding filter mask cell is unknown.
    // Do nothing.
    RCLCPP_ERROR(
      logger_,
      "SpeedFilter: Found unknown cell in filter_mask[%i, %i], "
      "which is invalid for this kind of filter",
      mask_robot_i, mask_robot_j);
    return false;
  } else {
    // Normal case: speed_mask_data in range of [1..100]
    speed_limit = speed_mask_data * multiplier_ + base_;
    if (percentage_) {
      if (speed_limit < 0.0 || speed_limit > 100.0) {
        RCLCPP_WARN(
          logger_,
          "SpeedFilter: Speed limit in filter_mask[%i, %i] is %f%%, "
          "out of bounds of [0, 100]. Setting it to no-limit value.",
          mask_robot_i, mask_robot_j, speed_limit);
        speed_limit = NO_SPEED_LIMIT;
      }
    } else {
      if (speed_limit < 0.0) {
        RCLCPP_WARN(
          logger_,
          "SpeedFilter: Speed limit in filter_mask[%i, %i] is less than 0 m/s, "
          "which can not be true. Setting it to no-limit value.",
          mask_robot_i, mask_robot_j);
        speed_limit = NO_SPEED_LIMIT;
      }
    }
  }
  return true;
}

double SpeedFilter::getSpeedLimitFromLookahead(
  const geometry_msgs::msg::Pose & robot_pose,
  double lookahead_dist)
{
  const auto & poses = current_path_->poses;

  const size_t pose_search_start =
    (cached_lookahead_start_idx_ < poses.size()) ? cached_lookahead_start_idx_ : 0;

  size_t lookahead_start_idx = pose_search_start;
  double min_dist = std::numeric_limits<double>::max();

  // Find the closest pose to the robot
  // To save computation time, stop searching path when poses get further than
  // the closest pose + a threshold
  for (size_t i = pose_search_start; i < poses.size(); i++) {
    const double d = nav2_util::geometry_utils::euclidean_distance(
      poses[i].pose.position, robot_pose.position);
    if (d < min_dist) {
      min_dist = d;
      lookahead_start_idx = i;
    } else if (d > min_dist + POSE_SEARCH_EXIT_THRESHOLD_) {
      break;
    }
  }

  // Update cached start index
  cached_lookahead_start_idx_ = lookahead_start_idx;

  double min_speed_limit = NO_SPEED_LIMIT;
  bool found_any_limit = false;

  // Lookahead endpoint for visualization
  auto lookahead_point_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  lookahead_point_msg->header.frame_id = global_frame_;
  lookahead_point_msg->header.stamp = clock_->now();
  lookahead_point_msg->point = robot_pose.position;

  // Check robot's current pose
  double speed_limit_at_robot_pose = NO_SPEED_LIMIT;
  if (!getSpeedLimitAtPose(robot_pose, speed_limit_at_robot_pose)) {
    // Pose mapped outside mask or transform failed
    return NO_SPEED_LIMIT;
  }
  if (speed_limit_at_robot_pose != NO_SPEED_LIMIT) {
    min_speed_limit = speed_limit_at_robot_pose;
    found_any_limit = true;
  }

  // Walk poses from lookahead_start_idx forward, sampling the speed limit at each pose.
  double dist_along_path = 0.0;
  for (size_t i = lookahead_start_idx; i < poses.size(); ++i) {
    if (i > lookahead_start_idx) {
      dist_along_path += nav2_util::geometry_utils::euclidean_distance(
        poses[i - 1].pose.position, poses[i].pose.position);
    }

    if (dist_along_path > lookahead_dist) {
      break;
    }

    // Update lookahead endpoint for visualization
    lookahead_point_msg->point = poses[i].pose.position;

    double sampled_speed_limit = NO_SPEED_LIMIT;
    if (!getSpeedLimitAtPose(poses[i].pose, sampled_speed_limit)) {
      // Pose mapped outside mask or transform failed
      continue;
    }
    if (sampled_speed_limit == NO_SPEED_LIMIT) {
      continue;
    }

    // Update strictest speed limit
    if (!found_any_limit) {
      // First limit found on the lookahead path, set it as the strictest
      min_speed_limit = sampled_speed_limit;
      found_any_limit = true;
    } else {
      min_speed_limit = std::min(min_speed_limit, sampled_speed_limit);
    }
  }

  if (lookahead_pub_ && lookahead_pub_->get_subscription_count() > 0) {
    lookahead_pub_->publish(std::move(lookahead_point_msg));
  }

  return min_speed_limit;
}

void SpeedFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "SpeedFilter: Filter mask was not received");
    return;
  }

  // Decide path lookahead vs just checking at robot pose.
  // Path lookahead requires a non-empty path received
  const bool use_path_lookahead =
    enable_path_lookahead_ &&
    current_path_ && !current_path_->poses.empty();

  if (use_path_lookahead) {
    const auto twist = odom_smoother_->getTwist();
    const double linear_vel = std::abs(twist.linear.x);

    // Calculate lookahead distance at current velocity
    double d_lookahead;
    if (max_decel_ < 0.0) {
      d_lookahead = (linear_vel * linear_vel) / (2.0 * std::abs(max_decel_));
      d_lookahead = std::clamp(d_lookahead, min_lookahead_, max_lookahead_);

      // If a limit is currently active, don't let the lookahead shrink below
      // the one that captured the speed zone
      if(speed_limit_ != NO_SPEED_LIMIT) {
        d_lookahead = std::max(d_lookahead, held_lookahead_dist_);
      }
    } else {
      d_lookahead = max_lookahead_;
    }

    speed_limit_ = getSpeedLimitFromLookahead(pose, d_lookahead);

    // If a limit is currently active, hold the lookahead distance
    if (speed_limit_ != NO_SPEED_LIMIT) {
      held_lookahead_dist_ = d_lookahead;
    } else {
      // If no limit is currently active, reset the held distance
      held_lookahead_dist_ = 0.0;
    }
  } else {
    if (!getSpeedLimitAtPose(pose, speed_limit_)) {
      RCLCPP_ERROR(logger_, "SpeedFilter: Failed to get speed limit at pose");
      return;
    }
  }

  if (speed_limit_ != speed_limit_prev_) {
    if (speed_limit_ != NO_SPEED_LIMIT) {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to %f", speed_limit_);
    } else {
      RCLCPP_DEBUG(logger_, "SpeedFilter: Speed limit is set to its default value");
    }

    // Forming and publishing new SpeedLimit message
    std::unique_ptr<nav2_msgs::msg::SpeedLimit> msg =
      std::make_unique<nav2_msgs::msg::SpeedLimit>();
    msg->header.frame_id = global_frame_;
    msg->header.stamp = clock_->now();
    msg->percentage = percentage_;
    msg->speed_limit = speed_limit_;
    speed_limit_pub_->publish(std::move(msg));

    speed_limit_prev_ = speed_limit_;
  }
}

void SpeedFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (speed_limit_pub_) {
    speed_limit_pub_->on_deactivate();
    speed_limit_pub_.reset();
  }
  if (lookahead_pub_) {
    lookahead_pub_->on_deactivate();
    lookahead_pub_.reset();
  }
  held_lookahead_dist_ = 0.0;
  cached_lookahead_start_idx_ = 0;
}

bool SpeedFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SpeedFilter, nav2_costmap_2d::Layer)
