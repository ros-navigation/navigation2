// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_RECOVERIES__RECOVERY_HPP_
#define NAV2_RECOVERIES__RECOVERY_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/collision_checker.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_recoveries
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using namespace std::chrono_literals;  //NOLINT

template<typename ActionT>
class Recovery
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  explicit Recovery(
    rclcpp::Node::SharedPtr & node, const std::string & recovery_name,
    std::shared_ptr<tf2_ros::Buffer> tf)
  : node_(node),
    recovery_name_(recovery_name),
    tf_(*tf),
    action_server_(nullptr),
    cycle_frequency_(10)
  {
    configure();
  }

  virtual ~Recovery()
  {
    cleanup();
  }

  // Derived classes can override this method to catch the command and perform some checks
  // before getting into the main loop. The method will only be called
  // once and should return SUCCEEDED otherwise behavior will return FAILED.
  virtual Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;


  // This is the method derived classes should mainly implement
  // and will be called cyclically while it returns RUNNING.
  // Implement the behavior such that it runs some unit of work on each call
  // and provides a status. The recovery will finish once SUCCEEDED is returned
  // It's up to the derived class to define the final commanded velocity.
  virtual Status onCycleUpdate() = 0;

protected:
  rclcpp::Node::SharedPtr node_;
  std::string recovery_name_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  tf2_ros::Buffer & tf_;
  std::unique_ptr<ActionServer> action_server_;

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::unique_ptr<nav2_costmap_2d::CollisionChecker> collision_checker_;
  double cycle_frequency_;

  void configure()
  {
    RCLCPP_INFO(node_->get_logger(), "Configuring %s", recovery_name_.c_str());

    std::string costmap_topic;
    std::string footprint_topic;

    node_->get_parameter("costmap_topic", costmap_topic);
    node_->get_parameter("footprint_topic", footprint_topic);

    action_server_ = std::make_unique<ActionServer>(node_, recovery_name_,
        std::bind(&Recovery::execute, this));

    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_, costmap_topic);

    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_, footprint_topic);

    collision_checker_ = std::make_unique<nav2_costmap_2d::CollisionChecker>(
      *costmap_sub_, *footprint_sub_, tf_, node_->get_name(), "odom");

    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }

  void cleanup()
  {
    action_server_.reset();
    vel_pub_.reset();
    footprint_sub_.reset();
    costmap_sub_.reset();
    collision_checker_.reset();
  }

  void execute()
  {
    RCLCPP_INFO(node_->get_logger(), "Attempting %s", recovery_name_.c_str());

    if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "Initial checks failed for %s", recovery_name_.c_str());
      action_server_->terminate_goals();
      return;
    }

    // Log a message every second
    auto timer = node_->create_wall_timer(1s,
        [&]() {RCLCPP_INFO(node_->get_logger(), "%s running...", recovery_name_.c_str());});

    rclcpp::Rate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(node_->get_logger(), "Canceling %s", recovery_name_.c_str());
        stopRobot();
        action_server_->terminate_goals();
        return;
      }

      // TODO(orduno) #868 Enable preempting a Recovery on-the-fly without stopping
      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(node_->get_logger(), "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          recovery_name_.c_str());
        stopRobot();
        action_server_->terminate_goals();
        return;
      }

      switch (onCycleUpdate()) {
        case Status::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "%s completed successfully", recovery_name_.c_str());
          action_server_->succeeded_current();
          return;

        case Status::FAILED:
          RCLCPP_WARN(node_->get_logger(), "%s failed", recovery_name_.c_str());
          action_server_->terminate_goals();
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub_->publish(cmd_vel);
  }
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__RECOVERY_HPP_
