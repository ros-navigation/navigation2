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

#ifndef NAV2_MOTION_PRIMITIVES__MOTION_PRIMITIVE_HPP_
#define NAV2_MOTION_PRIMITIVES__MOTION_PRIMITIVE_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_robot/robot.hpp"

namespace nav2_motion_primitives
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using namespace std::chrono_literals;  //NOLINT

template<typename ActionT>
class MotionPrimitive
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  explicit MotionPrimitive(rclcpp::Node::SharedPtr & node, const std::string & primitive_name)
  : node_(node),
    primitive_name_(primitive_name),
    action_server_(nullptr)
  {
    configure();
  }

  virtual ~MotionPrimitive()
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
  // and provides a status. The primitive will finish once SUCCEEDED is returned
  // It's up to the derived class to define the final commanded velocity.
  virtual Status onCycleUpdate() = 0;

protected:
  rclcpp::Node::SharedPtr node_;
  std::string primitive_name_;
  std::shared_ptr<nav2_robot::Robot> robot_;
  std::unique_ptr<ActionServer> action_server_;

  void configure()
  {
    RCLCPP_INFO(node_->get_logger(), "Configuring %s", primitive_name_.c_str());

    robot_ = std::make_unique<nav2_robot::Robot>(
      node_->get_node_base_interface(),
      node_->get_node_topics_interface(),
      node_->get_node_logging_interface(),
      true);

    action_server_ = std::make_unique<ActionServer>(node_, primitive_name_,
        std::bind(&MotionPrimitive::execute, this, std::placeholders::_1));
  }

  void cleanup()
  {
    robot_.reset();
    action_server_.reset();
  }

  void execute(const typename std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Attempting %s", primitive_name_.c_str());

    if (onRun(goal_handle->get_goal()) != Status::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "Initial checks failed for %s", primitive_name_.c_str());
      goal_handle->abort(std::make_shared<typename ActionT::Result>());
      return;
    }

    // Log a message every second
    auto timer = node_->create_wall_timer(1s,
        [&]() {RCLCPP_INFO(node_->get_logger(), "%s running...", primitive_name_.c_str());});

    auto result = std::make_shared<typename ActionT::Result>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(node_->get_logger(), "Canceling %s", primitive_name_.c_str());
        goal_handle->canceled(result);
        return;
      }

      switch (onCycleUpdate()) {
        case Status::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "%s completed successfully", primitive_name_.c_str());
          // Primitives actions results are empty msgs
          goal_handle->succeed(result);
          return;

        case Status::FAILED:
          RCLCPP_WARN(node_->get_logger(), "%s failed", primitive_name_.c_str());
          goal_handle->abort(result);
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }
};

}  // namespace nav2_motion_primitives

#endif  // NAV2_MOTION_PRIMITIVES__MOTION_PRIMITIVE_HPP_
