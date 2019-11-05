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

  start_reset_button_ = new QPushButton;
  pause_resume_button_ = new QPushButton;

  // Create the state machine used to present the proper control button states in the UI

  const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
  const char * shutdown_msg = "Deactivate and cleanup all nav2 lifecycle nodes";
  const char * cancel_msg = "Cancel navigation";
  const char * pause_msg = "Deactivate all nav2 lifecycle nodes";
  const char * resume_msg = "Activate all nav2 lifecycle nodes";

  pre_initial_ = new QState();
  pre_initial_->setObjectName("pre_initial");
  pre_initial_->assignProperty(start_reset_button_, "text", "Startup");
  pre_initial_->assignProperty(start_reset_button_, "enabled", false);

  pre_initial_->assignProperty(pause_resume_button_, "text", "Pause");
  pre_initial_->assignProperty(pause_resume_button_, "enabled", false);

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(start_reset_button_, "text", "Startup");
  initial_->assignProperty(start_reset_button_, "toolTip", startup_msg);
  initial_->assignProperty(start_reset_button_, "enabled", true);

  initial_->assignProperty(pause_resume_button_, "text", "Pause");
  initial_->assignProperty(pause_resume_button_, "enabled", false);

  // State entered when NavigateToPoses is not active
  idle_ = new QState();
  idle_->setObjectName("idle");
  idle_->assignProperty(start_reset_button_, "text", "Reset");
  idle_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);
  idle_->assignProperty(start_reset_button_, "enabled", true);

  idle_->assignProperty(pause_resume_button_, "text", "Pause");
  idle_->assignProperty(pause_resume_button_, "enabled", true);
  idle_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

  // State entered to cancel the NavigateToPoses action
  canceled_ = new QState();
  canceled_->setObjectName("canceled");

  // State entered to reset the nav2 lifecycle nodes
  reset_ = new QState();
  reset_->setObjectName("reset");

  // State entered while the NavigateToPoses action is active
  running_ = new QState();
  running_->setObjectName("running");
  running_->assignProperty(start_reset_button_, "text", "Cancel");
  running_->assignProperty(start_reset_button_, "toolTip", cancel_msg);

  running_->assignProperty(pause_resume_button_, "text", "Pause");
  running_->assignProperty(pause_resume_button_, "enabled", false);

  // State entered when pause is requested
  paused_ = new QState();
  paused_->setObjectName("pausing");
  paused_->assignProperty(start_reset_button_, "text", "Reset");
  paused_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);

  paused_->assignProperty(pause_resume_button_, "text", "Resume");
  paused_->assignProperty(pause_resume_button_, "toolTip", resume_msg);
  paused_->assignProperty(pause_resume_button_, "enabled", true);

  // State entered to resume the nav2 lifecycle nodes
  resumed_ = new QState();
  resumed_->setObjectName("resuming");

  QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
  QObject::connect(canceled_, SIGNAL(exited()), this, SLOT(onCancel()));
  QObject::connect(reset_, SIGNAL(exited()), this, SLOT(onShutdown()));
  QObject::connect(paused_, SIGNAL(entered()), this, SLOT(onPause()));
  QObject::connect(resumed_, SIGNAL(exited()), this, SLOT(onResume()));

  // Start/Reset button click transitions
  initial_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
  idle_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
  running_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
  paused_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);

  // Internal state transitions
  canceled_->addTransition(canceled_, SIGNAL(entered()), idle_);
  reset_->addTransition(reset_, SIGNAL(entered()), initial_);
  resumed_->addTransition(resumed_, SIGNAL(entered()), idle_);

  // Pause/Resume button click transitions
  idle_->addTransition(pause_resume_button_, SIGNAL(clicked()), paused_);
  paused_->addTransition(pause_resume_button_, SIGNAL(clicked()), resumed_);

  // ROSAction Transitions
  ROSActionQTransition * idleTransition = new ROSActionQTransition(QActionState::INACTIVE);
  idleTransition->setTargetState(running_);
  idle_->addTransition(idleTransition);

  ROSActionQTransition * runningTransition = new ROSActionQTransition(QActionState::ACTIVE);
  runningTransition->setTargetState(idle_);
  running_->addTransition(runningTransition);

  initial_thread_ = new InitialThread(client_);
  connect(initial_thread_, &InitialThread::finished, initial_thread_, &QObject::deleteLater);

  QSignalTransition * activeSignal = new QSignalTransition(initial_thread_,
      &InitialThread::activeSystem);
  activeSignal->setTargetState(idle_);
  pre_initial_->addTransition(activeSignal);

  QSignalTransition * inactiveSignal = new QSignalTransition(initial_thread_,
      &InitialThread::inactiveSystem);
  inactiveSignal->setTargetState(initial_);
  pre_initial_->addTransition(inactiveSignal);

  state_machine_.addState(pre_initial_);
  state_machine_.addState(initial_);
  state_machine_.addState(idle_);
  state_machine_.addState(running_);
  state_machine_.addState(canceled_);
  state_machine_.addState(reset_);
  state_machine_.addState(paused_);
  state_machine_.addState(resumed_);

  state_machine_.setInitialState(pre_initial_);

  // delay starting initial thread until state machine has started or a race occurs
  QObject::connect(&state_machine_, SIGNAL(started()), this, SLOT(startThread()));
  state_machine_.start();

  // Lay out the items in the panel
  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(pause_resume_button_);
  main_layout->addWidget(start_reset_button_);
  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=navigation_dialog_action_client"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPoses>(client_node_,
      "NavigateToPoses");
  goal_ = nav2_msgs::action::NavigateToPoses::Goal();

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
Nav2Panel::startThread()
{
  // start initial thread now that state machine is started
  initial_thread_->start();
}

void
Nav2Panel::onPause()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::pause, &client_));
}

void
Nav2Panel::onResume()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::resume, &client_));
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
    QtConcurrent::run(std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::reset,
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

  auto poses = nav_msgs::msg::Path();
  poses.poses.push_back(pose);
  startNavigation(poses);
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
Nav2Panel::startNavigation(nav_msgs::msg::Path poses)
{
  auto is_action_server_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(client_node_->get_logger(), "NavigateToPoses action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  goal_.poses = poses;

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPoses>::SendGoalOptions();
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
