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
#include "nav2_tasks/task_status.hpp"
#include "nav2_tasks/task_server.hpp"
#include "nav2_robot/robot.hpp"

namespace nav2_motion_primitives
{

using namespace std::chrono_literals;  //NOLINT

template<class CommandMsg, class ResultMsg>
const char * getTaskName();

template<class CommandMsg, class ResultMsg>
class MotionPrimitive
{
public:
  explicit MotionPrimitive(rclcpp::Node::SharedPtr & node)
  : node_(node),
    task_server_(nullptr),
    taskName_(nav2_tasks::getTaskName<CommandMsg, ResultMsg>())
  {
    robot_ = std::make_unique<nav2_robot::Robot>(node);

    task_server_ = std::make_unique<nav2_tasks::TaskServer<CommandMsg, ResultMsg>>(node, false);

    task_server_->setExecuteCallback(std::bind(&MotionPrimitive::run, this, std::placeholders::_1));

    // Start listening for incoming Spin task requests
    task_server_->startWorkerThread();

    RCLCPP_INFO(node_->get_logger(), "Initialized the %s server", taskName_.c_str());
  }

  virtual ~MotionPrimitive() {}

  // Derived classes can override this method to catch the command and perform some checks
  // before getting into the main loop. The method will only be called
  // once and should return SUCCEEDED otherwise behavior will return FAILED.
  virtual nav2_tasks::TaskStatus onRun(
    const typename /*nav2_tasks::*/ CommandMsg::SharedPtr command) = 0;

  // This is the method derived classes should mainly implement
  // and will be called cyclically while it returns RUNNING.
  // Implement the behavior such that it runs some unit of work on each call
  // and provides a status.
  virtual nav2_tasks::TaskStatus onCycleUpdate(
    /*nav2_tasks::*/ ResultMsg & result) = 0;

  // Runs the behavior
  nav2_tasks::TaskStatus run(
    const typename /*nav2_tasks::*/ CommandMsg::SharedPtr command)
  {
    RCLCPP_INFO(node_->get_logger(), "%s attempting behavior", taskName_.c_str());

    ResultMsg result;
    auto status = onRun(command);

    if (status == nav2_tasks::TaskStatus::SUCCEEDED) {
      status = cycle(result);
    }

    task_server_->setResult(result);

    return status;
  }

protected:
  nav2_tasks::TaskStatus cycle(ResultMsg & result)
  {
    auto time_since_msg = std::chrono::system_clock::now();
    auto start_time = std::chrono::system_clock::now();
    auto current_time = std::chrono::system_clock::now();

    auto status = nav2_tasks::TaskStatus::FAILED;

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      if (task_server_->cancelRequested()) {
        RCLCPP_INFO(node_->get_logger(), "%s cancelled", taskName_.c_str());
        task_server_->setCanceled();
        status = nav2_tasks::TaskStatus::CANCELED;
        break;
      }

      // Log a message every second
      current_time = std::chrono::system_clock::now();
      if (current_time - time_since_msg >= 1s) {
        RCLCPP_INFO(node_->get_logger(), "%s running...", taskName_.c_str());
        time_since_msg = std::chrono::system_clock::now();
      }

      status = onCycleUpdate(result);

      if (status == nav2_tasks::TaskStatus::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "%s completed successfully", taskName_.c_str());
        break;
      }

      if (status == nav2_tasks::TaskStatus::FAILED) {
        RCLCPP_WARN(node_->get_logger(), "%s was not completed", taskName_.c_str());
        break;
      }

      if (status == nav2_tasks::TaskStatus::CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "%s onCycleUpdate() should not check for"
          " task cancellation, it will be checked by the base class.", taskName_.c_str());
        break;
      }
      loop_rate.sleep();
    }

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;

    RCLCPP_INFO(node_->get_logger(), "%s ran for %.2f seconds",
      taskName_.c_str(), elapsed_seconds.count());

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    robot_->sendVelocity(twist);

    return status;
  }

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<nav2_robot::Robot> robot_;

  typename std::unique_ptr<nav2_tasks::TaskServer<CommandMsg, ResultMsg>> task_server_;

  std::string taskName_;
};

}  // namespace nav2_motion_primitives

#endif  // NAV2_MOTION_PRIMITIVES__MOTION_PRIMITIVE_HPP_
