
// Copyright (c) 2021 Anushree Sabnis
// Copyright (c) 2022 Joshua Wallace
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

#include "nav2_assisted_teleop/nav2_assisted_teleop.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/node_utils.hpp"

#include <thread>

namespace nav2_assisted_teleop
{

  AssistedTeleop::AssistedTeleop(rclcpp::Logger& logger) : logger_(logger)
  {}

  void AssistedTeleop::on_configure(std::shared_ptr<nav2_util::LifecycleNode> node,
                    std::shared_ptr<tf2_ros::Buffer> tf,
                    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker)
  {
    // set up parameters
    node->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));

    nav2_util::declare_parameter_if_not_declared(
    node, "projection_time", rclcpp::ParameterValue(1.0));

    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("robot_base_frame", robot_base_frame_);

    // Note: empty frames don't cause robot_utils to error
    // nav2_util::declare_parameter_if_not_declared(
    //   node,
    //   "global_frame", rclcpp::ParameterValue(std::string("global_frame")));

    // nav2_util::declare_parameter_if_not_declared(
    //   node,
    //   "robot_base_frame", rclcpp::ParameterValue(std::string("robot_base_frame")));

    tf_ = tf;
    collision_checker_ = collision_checker;
  }

  geometry_msgs::msg::Twist AssistedTeleop::computeVelocity(geometry_msgs::msg::Twist& twist)
  {
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      RCLCPP_ERROR(logger_, "Failed to get current pose, setting velocity to zero");
      return geometry_msgs::msg::Twist();
    }

    geometry_msgs::msg::Pose2D current_pose_2d;
    current_pose_2d.x = current_pose.pose.position.x;
    current_pose_2d.y = current_pose.pose.position.y;
    current_pose_2d.theta = 0.0; //tf2::getYaw(current_pose.pose.orientation);

    const double linear_vel = std::fabs(std::hypot(twist.linear.x, twist.linear.y));
    const double dt = 0.05 / linear_vel;

    geometry_msgs::msg::Pose2D projected_pose = current_pose_2d;
    auto adjusted_twist = twist;
    for(double current_time = 0; current_time < projection_time_; current_time+=dt)
    {
      projected_pose = projectPose(projected_pose, twist, dt);

      if(!collision_checker_->isCollisionFree(projected_pose))
      {
        adjusted_twist.linear.x *= current_time / projection_time_;
        adjusted_twist.linear.y *= current_time / projection_time_;
        adjusted_twist.angular.z *= current_time / projection_time_;

        RCLCPP_WARN_STREAM(logger_, "Collision approaching in " << current_time << ". Reducing velocity");
      }
    }
    return adjusted_twist;
  }

  geometry_msgs::msg::Pose2D AssistedTeleop::projectPose(geometry_msgs::msg::Pose2D pose,
                                          geometry_msgs::msg::Twist twist,
                                          double projection_time)
  {
    geometry_msgs::msg::Pose2D projected_pose;

    projected_pose.x = projection_time * (
                                twist.linear.x * cos(pose.theta) +
                                twist.linear.y * sin(pose.theta));

    projected_pose.x = projection_time * (
                                twist.linear.x * sin(pose.theta) -
                                twist.linear.y * cos(pose.theta));

    projected_pose.theta = projection_time*twist.angular.z;

    return projected_pose;
  }

  AssistedTeleopServer::AssistedTeleopServer(const rclcpp::NodeOptions & options)
  : LifecycleNode("assisted_teleop_server", "", false, options),
  logger_(get_logger()),
  assisted_teleop_(std::make_shared<AssistedTeleop>(logger_))
  {
    RCLCPP_INFO(logger_, "Creating assisted teleop server");
  }

  nav2_util::CallbackReturn
  AssistedTeleopServer::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "Configuring assisted_teleop_server");

    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(
      this,
      "input_vel_topic", rclcpp::ParameterValue(std::string("input_vel_topic")));

    nav2_util::declare_parameter_if_not_declared(
      node,
      "output_vel_topic", rclcpp::ParameterValue(std::string("cmd_vel")));

    nav2_util::declare_parameter_if_not_declared(
      node,
      "joystick_topic", rclcpp::ParameterValue(std::string("joystick_topic")));

    declare_parameter(
    "costmap_topic", rclcpp::ParameterValue(
      std::string(
        "global_costmap/costmap_raw")));

    declare_parameter(
    "footprint_topic",
    rclcpp::ParameterValue(
      std::string("global_costmap/published_footprint")));

    // get parameters
    this->get_parameter("transform_tolerance", transform_tolerance_);
    this->get_parameter("projection_time", projection_time_);
    this->get_parameter("input_vel_topic", input_vel_topic_);
    this->get_parameter("output_vel_topic", output_vel_topic_);
    this->get_parameter("joystick_topic", joystick_topic_);

    std::string costmap_topic, footprint_topic;
    this->get_parameter("costmap_topic", costmap_topic);
    this->get_parameter("footprint_topic", footprint_topic);

    // setup transform manager
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);

    // setup collision checker
    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), costmap_topic);
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
    shared_from_this(), footprint_topic, *tf_, robot_base_frame_, transform_tolerance_);

    collision_checker_ =
    std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, this->get_name());


    // TODO (jwallace42): node should be passed in as weakptr?
    assisted_teleop_->on_configure(node, tf_, collision_checker_);

    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "assisted_teleop",
      std::bind(&AssistedTeleopServer::adjustVelocityIfInvalid, this),
      nullptr,
      std::chrono::milliseconds(500),
      true);

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_vel_topic_, 1);

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  AssistedTeleopServer::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "Activating assisted_teleop_server");
    vel_pub_->on_activate();
    action_server_->activate();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  AssistedTeleopServer::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "Deactivating assisted_teleop_server");

    vel_pub_->on_deactivate();
    action_server_->deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  AssistedTeleopServer::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "Cleaning up assisted_teleop_server");

    vel_pub_->on_activate();
    action_server_->activate();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  void AssistedTeleopServer::adjustVelocityIfInvalid()
  {
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleopServer::inputVelocityCallback,
      this, std::placeholders::_1));

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    joystick_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleopServer::joyCallback,
      this, std::placeholders::_1));

    auto start_time = steady_clock_.now();

    auto result = std::make_shared<Action::Result>();
    rclcpp::WallRate loop_rate(10);
    while (rclcpp::ok())
    {
      RCLCPP_INFO(logger_, "Running");
      // user states that teleop was successful
      if( joy_.buttons.size() > 0 && joy_.buttons[0] > 0)
      {
        vel_pub_->publish(geometry_msgs::msg::Twist());
        vel_sub_.reset();
        joy_sub_.reset();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->succeeded_current(result);
        return;
      }

      // user states that teleop failed
      if( joy_.buttons.size() > 1 && joy_.buttons[1] > 0)
      {
        vel_pub_->publish(geometry_msgs::msg::Twist());
        vel_sub_.reset();
        joy_sub_.reset();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->terminate_all(result);
        return;
      }

      // cancel request
      if(action_server_->is_cancel_requested())
      {
        vel_pub_->publish(geometry_msgs::msg::Twist());
        vel_sub_.reset();
        joy_sub_.reset();
        result->total_elapsed_time = steady_clock_.now() -start_time;
        action_server_->terminate_all(result);
        return;
      }

      // preempt request
      if (action_server_->is_preempt_requested()) {
        vel_pub_->publish(geometry_msgs::msg::Twist());
        vel_sub_.reset();
        joy_sub_.reset();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->terminate_current(result);
        return;
      }

      geometry_msgs::msg::Twist cmd_vel;

      cmd_vel = assisted_teleop_->computeVelocity(input_twist_);

      vel_pub_->publish(std::move(cmd_vel));
      loop_rate.sleep();
    }
  }

  void AssistedTeleopServer::inputVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(logger_, "Hit input Vel callback");

    input_twist_.linear.x = msg->linear.x;
    input_twist_.linear.y = msg->linear.y;
    input_twist_.angular.z = msg->linear.z;
  }

  void AssistedTeleopServer::joyCallback(const sensor_msgs::msg::Joy msg)
  {
    RCLCPP_INFO(logger_, "Hit joy callback");
    joy_.buttons = msg.buttons;
  }
}  // nav2_assisted_teleop
