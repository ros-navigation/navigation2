// Copyright (c) 2019 Intel Corporation
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

#include "nav2_rviz_plugins/nav2_panel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>

#include "nav2_rviz_plugins/goal_common.hpp"
#include "rviz_common/display_context.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{
using nav2_util::geometry_utils::orientationAroundZAxis;

// Define global GoalPoseUpdater so that the nav2 GoalTool plugin can access to update goal pose
GoalPoseUpdater GoalUpdater;

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent)
{
  // Create the control button and its tooltip

  start_stop_button_ = new QPushButton;

  // Create the state machine used to present the proper control button states in the UI

  const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
  const char * shutdown_msg = "Deactivate, cleanup, and shutdown all nav2 lifecycle nodes";
  const char * cancel_msg = "Cancel navigation";

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(start_stop_button_, "text", "Startup");
  initial_->assignProperty(start_stop_button_, "toolTip", startup_msg);

  starting_ = new QState();
  starting_->setObjectName("starting");
  starting_->assignProperty(start_stop_button_, "text", "Shutdown");
  starting_->assignProperty(start_stop_button_, "toolTip", shutdown_msg);

  // State entered after NavigateToPose has been canceled
  canceled_ = new QState();
  canceled_->setObjectName("canceled");
  canceled_->assignProperty(start_stop_button_, "text", "Shutdown");
  canceled_->assignProperty(start_stop_button_, "toolTip", shutdown_msg);

  // State entered after the NavigateToPose action has completed
  completed_ = new QState();
  completed_->setObjectName("succesful");
  completed_->assignProperty(start_stop_button_, "text", "Shutdown");
  completed_->assignProperty(start_stop_button_, "toolTip", shutdown_msg);

  // State entered while the NavigateToPose action is active
  running_ = new QState();
  running_->setObjectName("running");
  running_->assignProperty(start_stop_button_, "text", "Cancel");
  running_->assignProperty(start_stop_button_, "toolTip", cancel_msg);

  stopping_ = new QState();
  stopping_->setObjectName("stopping");
  stopping_->assignProperty(start_stop_button_, "enabled", false);

  QObject::connect(starting_, SIGNAL(entered()), this, SLOT(onStartup()));
  QObject::connect(stopping_, SIGNAL(entered()), this, SLOT(onShutdown()));
  QObject::connect(canceled_, SIGNAL(entered()), this, SLOT(onCancel()));

  initial_->addTransition(start_stop_button_, SIGNAL(clicked()), starting_);
  starting_->addTransition(start_stop_button_, SIGNAL(clicked()), stopping_);
  running_->addTransition(start_stop_button_, SIGNAL(clicked()), canceled_);
  canceled_->addTransition(start_stop_button_, SIGNAL(clicked()), stopping_);
  completed_->addTransition(start_stop_button_, SIGNAL(clicked()), stopping_);

  ROSActionQTransition * startupTransition = new ROSActionQTransition(QActionState::INACTIVE);
  startupTransition->setTargetState(running_);
  starting_->addTransition(startupTransition);

  ROSActionQTransition * canceledTransition = new ROSActionQTransition(QActionState::INACTIVE);
  canceledTransition->setTargetState(running_);
  canceled_->addTransition(canceledTransition);

  ROSActionQTransition * runningTransition = new ROSActionQTransition(QActionState::ACTIVE);
  runningTransition->setTargetState(completed_);
  running_->addTransition(runningTransition);

  ROSActionQTransition * completedTransition = new ROSActionQTransition(QActionState::INACTIVE);
  completedTransition->setTargetState(running_);
  completed_->addTransition(completedTransition);

  state_machine_.addState(initial_);
  state_machine_.addState(starting_);
  state_machine_.addState(stopping_);
  state_machine_.addState(running_);
  state_machine_.addState(canceled_);
  state_machine_.addState(completed_);

  state_machine_.setInitialState(initial_);
  state_machine_.start();

  // Lay out the items in the panel
  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(start_stop_button_);
  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);

  auto options = rclcpp::NodeOptions().arguments(
    {"__node:=navigation_dialog_action_client"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(client_node_,
      "NavigateToPose");
  goal_ = nav2_msgs::action::NavigateToPose::Goal();

  QObject::connect(&GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),  // NOLINT
    this, SLOT(onNewGoal(double,double,double,QString)));  // NOLINT
}

Nav2Panel::~Nav2Panel()
{
}

void
Nav2Panel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void
Nav2Panel::onStartup()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::startup,
      &client_));
}

void
Nav2Panel::onShutdown()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::shutdown,
      &client_));

  timer_.stop();
}

void
Nav2Panel::onCancel()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&Nav2Panel::onCancelButtonPressed,
      this));
}

void
Nav2Panel::onNewGoal(double x, double y, double theta, QString frame)
{
  auto pose = geometry_msgs::msg::PoseStamped();

  pose.header.stamp = rclcpp::Clock().now();
  pose.header.frame_id = frame.toStdString();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = orientationAroundZAxis(theta);

  startNavigation(pose);
}

void
Nav2Panel::onCancelButtonPressed()
{
  auto future_cancel = action_client_->async_cancel_goal(goal_handle_);

  if (rclcpp::spin_until_future_complete(client_node_, future_cancel) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    return;
  }

  timer_.stop();
}

void
Nav2Panel::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_.timerId()) {
    if (!goal_handle_) {
      RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
      state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      return;
    }

    rclcpp::spin_some(client_node_);
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    {
      state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    } else {
      state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      timer_.stop();
    }
  }
}

void
Nav2Panel::startNavigation(geometry_msgs::msg::PoseStamped pose)
{
  auto is_action_server_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(client_node_->get_logger(), "NavigateToPose action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal pose
  goal_.pose = pose;

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [](auto) {};

  auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }

  timer_.start(200, this);
}

void
Nav2Panel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void
Nav2Panel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Nav2Panel, rviz_common::Panel)
