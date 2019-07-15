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
#include "nav2_costmap_2d/collision_checker.hpp"
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
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  explicit MotionPrimitive(rclcpp::Node::SharedPtr & node, const std::string & primitive_name)
  : node_(node),
    primitive_name_(primitive_name),
    action_server_(nullptr),
    cycle_frequency_(10)
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
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  nav2_util::GetRobotPoseClient get_robot_pose_client_{"motion_primitive"};
  std::unique_ptr<nav2_costmap_2d::CollisionChecker> collision_checker_;
  double cycle_frequency_;

  void configure()
  {
    RCLCPP_INFO(node_->get_logger(), "Configuring %s", primitive_name_.c_str());

    std::string costmap_topic;
    std::string footprint_topic;

    node_->get_parameter("costmap_topic", costmap_topic);
    node_->get_parameter("footprint_topic", footprint_topic);

    robot_ = std::make_unique<nav2_robot::Robot>(
      node_->get_node_base_interface(),
      node_->get_node_topics_interface(),
      node_->get_node_logging_interface(),
      true);

    action_server_ = std::make_unique<ActionServer>(node_, primitive_name_,
        std::bind(&MotionPrimitive::execute, this));

    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_, costmap_topic);

    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_, footprint_topic);

    collision_checker_ = std::make_unique<nav2_costmap_2d::CollisionChecker>(
      *costmap_sub_, *footprint_sub_, get_robot_pose_client_);
  }

  void cleanup()
  {
    robot_.reset();
    action_server_.reset();

    footprint_sub_.reset();
    costmap_sub_.reset();
    collision_checker_.reset();
  }

  void execute()
  {
    RCLCPP_INFO(node_->get_logger(), "Attempting %s", primitive_name_.c_str());

    if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "Initial checks failed for %s", primitive_name_.c_str());
      action_server_->terminate_goals();
      return;
    }

    // Log a message every second
    auto timer = node_->create_wall_timer(1s,
        [&]() {RCLCPP_INFO(node_->get_logger(), "%s running...", primitive_name_.c_str());});

    rclcpp::Rate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(node_->get_logger(), "Canceling %s", primitive_name_.c_str());
        action_server_->terminate_goals();
        return;
      }

      // TODO(orduno) #868 Enable preempting a motion primitive on-the-fly without stopping
      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(node_->get_logger(), "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          primitive_name_.c_str());
        stopRobot();
        action_server_->terminate_goals();
        return;
      }

      switch (onCycleUpdate()) {
        case Status::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "%s completed successfully", primitive_name_.c_str());
          action_server_->succeeded_current();
          return;

        case Status::FAILED:
          RCLCPP_WARN(node_->get_logger(), "%s failed", primitive_name_.c_str());
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

    robot_->sendVelocity(cmd_vel);
  }

  bool getRobotPose(geometry_msgs::msg::Pose & current_pose)
  {
    auto request = std::make_shared<nav2_util::GetRobotPoseClient::GetRobotPoseRequest>();

    auto result = get_robot_pose_client_.invoke(request, 1s);
    if (!result->is_pose_valid) {
      return false;
    }
    current_pose = result->pose.pose;
    return true;
  }
};

}  // namespace nav2_motion_primitives

#endif  // NAV2_MOTION_PRIMITIVES__MOTION_PRIMITIVE_HPP_
