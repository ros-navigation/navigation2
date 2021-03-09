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
#include <vector>
#include <utility>

#include "nav2_rviz_plugins/goal_common.hpp"
#include "rviz_common/display_context.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{
using nav2_util::geometry_utils::orientationAroundZAxis;

// Define global GoalPoseUpdater so that the nav2 GoalTool plugin can access to update goal pose
GoalPoseUpdater GoalUpdater;

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent),
  server_timeout_(10),
  client_nav_("lifecycle_manager_navigation"),
  client_loc_("lifecycle_manager_localization")
{
  // Create the control button and its tooltip

  start_reset_button_ = new QPushButton;
  pause_resume_button_ = new QPushButton;
  navigation_mode_button_ = new QPushButton;
  navigation_status_indicator_ = new QLabel;
  localization_status_indicator_ = new QLabel;

  // Create the state machine used to present the proper control button states in the UI

  const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
  const char * shutdown_msg = "Deactivate and cleanup all nav2 lifecycle nodes";
  const char * cancel_msg = "Cancel navigation";
  const char * pause_msg = "Deactivate all nav2 lifecycle nodes";
  const char * resume_msg = "Activate all nav2 lifecycle nodes";
  const char * single_goal_msg = "Change to waypoint mode navigation";
  const char * waypoint_goal_msg = "Start navigation";
  const char * cancel_waypoint_msg = "Cancel waypoint mode";

  const QString navigation_active("<table><tr><td width=100><b>Navigation:</b></td>"
    "<td><font color=green>active</color></td></tr></table>");
  const QString navigation_inactive("<table><tr><td width=100><b>Navigation:</b></td>"
    "<td>inactive</td></tr></table>");
  const QString navigation_unknown("<table><tr><td width=100><b>Navigation:</b></td>"
    "<td>unknown</td></tr></table>");
  const QString localization_active("<table><tr><td width=100><b>Localization:</b></td>"
    "<td><font color=green>active</color></td></tr></table>");
  const QString localization_inactive("<table><tr><td width=100><b>Localization:</b></td>"
    "<td>inactive</td></tr></table>");
  const QString localization_unknown("<table><tr><td width=100><b>Localization:</b></td>"
    "<td>unknown</td></tr></table>");

  navigation_status_indicator_->setText(navigation_unknown);
  localization_status_indicator_->setText(localization_unknown);
  navigation_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  localization_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  pre_initial_ = new QState();
  pre_initial_->setObjectName("pre_initial");
  pre_initial_->assignProperty(start_reset_button_, "text", "Startup");
  pre_initial_->assignProperty(start_reset_button_, "enabled", false);

  pre_initial_->assignProperty(pause_resume_button_, "text", "Pause");
  pre_initial_->assignProperty(pause_resume_button_, "enabled", false);

  pre_initial_->assignProperty(navigation_mode_button_, "text", "Waypoint mode");
  pre_initial_->assignProperty(navigation_mode_button_, "enabled", false);

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(start_reset_button_, "text", "Startup");
  initial_->assignProperty(start_reset_button_, "toolTip", startup_msg);
  initial_->assignProperty(start_reset_button_, "enabled", true);

  initial_->assignProperty(pause_resume_button_, "text", "Pause");
  initial_->assignProperty(pause_resume_button_, "enabled", false);

  initial_->assignProperty(navigation_mode_button_, "text", "Waypoint mode");
  initial_->assignProperty(navigation_mode_button_, "enabled", false);

  // State entered when navigate_to_pose action is not active
  idle_ = new QState();
  idle_->setObjectName("idle");
  idle_->assignProperty(start_reset_button_, "text", "Reset");
  idle_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);
  idle_->assignProperty(start_reset_button_, "enabled", true);

  idle_->assignProperty(pause_resume_button_, "text", "Pause");
  idle_->assignProperty(pause_resume_button_, "enabled", true);
  idle_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

  idle_->assignProperty(navigation_mode_button_, "text", "Waypoint mode");
  idle_->assignProperty(navigation_mode_button_, "enabled", true);
  idle_->assignProperty(navigation_mode_button_, "toolTip", single_goal_msg);

  // State entered when navigate_to_pose action is not active
  accumulating_ = new QState();
  accumulating_->setObjectName("accumulating");
  accumulating_->assignProperty(start_reset_button_, "text", "Cancel Waypoint Mode");
  accumulating_->assignProperty(start_reset_button_, "toolTip", cancel_waypoint_msg);
  accumulating_->assignProperty(start_reset_button_, "enabled", true);

  accumulating_->assignProperty(pause_resume_button_, "text", "Pause");
  accumulating_->assignProperty(pause_resume_button_, "enabled", false);
  accumulating_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

  accumulating_->assignProperty(navigation_mode_button_, "text", "Start Navigation");
  accumulating_->assignProperty(navigation_mode_button_, "enabled", true);
  accumulating_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

  accumulated_ = new QState();
  accumulated_->setObjectName("accumulated");
  accumulated_->assignProperty(start_reset_button_, "text", "Cancel");
  accumulated_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
  accumulated_->assignProperty(start_reset_button_, "enabled", true);

  accumulated_->assignProperty(pause_resume_button_, "text", "Pause");
  accumulated_->assignProperty(pause_resume_button_, "enabled", false);
  accumulated_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

  accumulated_->assignProperty(navigation_mode_button_, "text", "Start Navigation");
  accumulated_->assignProperty(navigation_mode_button_, "enabled", false);
  accumulated_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

  // State entered to cancel the navigate_to_pose action
  canceled_ = new QState();
  canceled_->setObjectName("canceled");

  // State entered to reset the nav2 lifecycle nodes
  reset_ = new QState();
  reset_->setObjectName("reset");

  // State entered while the navigate_to_pose action is active
  running_ = new QState();
  running_->setObjectName("running");
  running_->assignProperty(start_reset_button_, "text", "Cancel");
  running_->assignProperty(start_reset_button_, "toolTip", cancel_msg);

  running_->assignProperty(pause_resume_button_, "text", "Pause");
  running_->assignProperty(pause_resume_button_, "enabled", false);

  running_->assignProperty(navigation_mode_button_, "text", "Waypoint mode");
  running_->assignProperty(navigation_mode_button_, "enabled", false);

  // State entered when pause is requested
  paused_ = new QState();
  paused_->setObjectName("pausing");
  paused_->assignProperty(start_reset_button_, "text", "Reset");
  paused_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);

  paused_->assignProperty(pause_resume_button_, "text", "Resume");
  paused_->assignProperty(pause_resume_button_, "toolTip", resume_msg);
  paused_->assignProperty(pause_resume_button_, "enabled", true);

  paused_->assignProperty(navigation_mode_button_, "text", "Start navigation");
  paused_->assignProperty(navigation_mode_button_, "toolTip", resume_msg);
  paused_->assignProperty(navigation_mode_button_, "enabled", true);

  // State entered to resume the nav2 lifecycle nodes
  resumed_ = new QState();
  resumed_->setObjectName("resuming");

  QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
  QObject::connect(canceled_, SIGNAL(exited()), this, SLOT(onCancel()));
  QObject::connect(reset_, SIGNAL(exited()), this, SLOT(onShutdown()));
  QObject::connect(paused_, SIGNAL(entered()), this, SLOT(onPause()));
  QObject::connect(resumed_, SIGNAL(exited()), this, SLOT(onResume()));
  QObject::connect(accumulating_, SIGNAL(entered()), this, SLOT(onAccumulating()));
  QObject::connect(accumulated_, SIGNAL(entered()), this, SLOT(onAccumulated()));

  // Start/Reset button click transitions
  initial_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
  idle_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
  running_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
  paused_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
  idle_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulating_);
  accumulating_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulated_);
  accumulating_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
  accumulated_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);

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

  ROSActionQTransition * idleAccumulatedTransition =
    new ROSActionQTransition(QActionState::INACTIVE);
  idleAccumulatedTransition->setTargetState(accumulated_);
  idle_->addTransition(idleAccumulatedTransition);

  ROSActionQTransition * accumulatedTransition = new ROSActionQTransition(QActionState::ACTIVE);
  accumulatedTransition->setTargetState(idle_);
  accumulated_->addTransition(accumulatedTransition);

  initial_thread_ = new InitialThread(client_nav_, client_loc_);
  connect(initial_thread_, &InitialThread::finished, initial_thread_, &QObject::deleteLater);

  QSignalTransition * activeSignal = new QSignalTransition(
    initial_thread_,
    &InitialThread::navigationActive);
  activeSignal->setTargetState(idle_);
  pre_initial_->addTransition(activeSignal);

  QSignalTransition * inactiveSignal = new QSignalTransition(
    initial_thread_,
    &InitialThread::navigationInactive);
  inactiveSignal->setTargetState(initial_);
  pre_initial_->addTransition(inactiveSignal);

  QObject::connect(
    initial_thread_, &InitialThread::navigationActive,
    [this, navigation_active] {
      navigation_status_indicator_->setText(navigation_active);
    });
  QObject::connect(
    initial_thread_, &InitialThread::navigationInactive,
    [this, navigation_inactive] {
      navigation_status_indicator_->setText(navigation_inactive);
    });
  QObject::connect(
    initial_thread_, &InitialThread::localizationActive,
    [this, localization_active] {
      localization_status_indicator_->setText(localization_active);
    });
  QObject::connect(
    initial_thread_, &InitialThread::localizationInactive,
    [this, localization_inactive] {
      localization_status_indicator_->setText(localization_inactive);
    });

  state_machine_.addState(pre_initial_);
  state_machine_.addState(initial_);
  state_machine_.addState(idle_);
  state_machine_.addState(running_);
  state_machine_.addState(canceled_);
  state_machine_.addState(reset_);
  state_machine_.addState(paused_);
  state_machine_.addState(resumed_);
  state_machine_.addState(accumulating_);
  state_machine_.addState(accumulated_);

  state_machine_.setInitialState(pre_initial_);

  // delay starting initial thread until state machine has started or a race occurs
  QObject::connect(&state_machine_, SIGNAL(started()), this, SLOT(startThread()));
  state_machine_.start();

  // Lay out the items in the panel
  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(navigation_status_indicator_);
  main_layout->addWidget(localization_status_indicator_);
  main_layout->addWidget(pause_resume_button_);
  main_layout->addWidget(start_reset_button_);
  main_layout->addWidget(navigation_mode_button_);

  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=navigation_dialog_action_client"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  navigation_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_,
    "navigate_to_pose");
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    client_node_,
    "FollowWaypoints");
  navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
  waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

  wp_navigation_markers_pub_ =
    client_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "waypoints",
    rclcpp::QoS(1).transient_local());

  QObject::connect(
    &GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)),                 // NOLINT
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
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::pause,
      &client_nav_, std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::pause,
      &client_loc_, std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onResume()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::resume,
      &client_nav_, std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::resume,
      &client_loc_, std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onStartup()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::startup,
      &client_nav_, std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::startup,
      &client_loc_, std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onShutdown()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::reset,
      &client_nav_, std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::reset,
      &client_loc_, std::placeholders::_1), server_timeout_);
  timer_.stop();
}

void
Nav2Panel::onCancel()
{
  QFuture<void> future =
    QtConcurrent::run(
    std::bind(
      &Nav2Panel::onCancelButtonPressed,
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

  if (state_machine_.configuration().contains(accumulating_)) {
    acummulated_poses_.push_back(pose);
  } else {
    std::cout << "Start navigation" << std::endl;
    startNavigation(pose);
  }

  updateWpNavigationMarkers();
}

void
Nav2Panel::onCancelButtonPressed()
{
  if (waypoint_follower_goal_handle_) {
    auto future_cancel =
      waypoint_follower_action_client_->async_cancel_goal(waypoint_follower_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel waypoint follower");
      return;
    }
  }

  if (navigation_goal_handle_) {
    auto future_cancel = navigation_action_client_->async_cancel_goal(navigation_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
      return;
    }
  }

  timer_.stop();
}

void
Nav2Panel::onAccumulated()
{
  std::cout << "Start waypoint" << std::endl;
  startWaypointFollowing(acummulated_poses_);
  acummulated_poses_.clear();
}

void
Nav2Panel::onAccumulating()
{
  acummulated_poses_.clear();
}

void
Nav2Panel::timerEvent(QTimerEvent * event)
{
  if (state_machine_.configuration().contains(accumulated_)) {
    if (event->timerId() == timer_.timerId()) {
      if (!waypoint_follower_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = waypoint_follower_goal_handle_->get_status();

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
  } else {
    if (event->timerId() == timer_.timerId()) {
      if (!navigation_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = navigation_goal_handle_->get_status();

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
}

void
Nav2Panel::startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(), "FollowWaypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_INFO(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    waypoint_follower_goal_.poses.size());
  for (auto waypoint : waypoint_follower_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback = [](auto) {};

  auto future_goal_handle =
    waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }

  timer_.start(200, this);
}

void
Nav2Panel::startNavigation(geometry_msgs::msg::PoseStamped pose)
{
  auto is_action_server_ready =
    navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "FollowWaypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal pose
  navigation_goal_.pose = pose;

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [](auto) {};

  auto future_goal_handle =
    navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  navigation_goal_handle_ = future_goal_handle.get();
  if (!navigation_goal_handle_) {
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

void
Nav2Panel::resetUniqueId()
{
  unique_id = 0;
}

int
Nav2Panel::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}

void
Nav2Panel::updateWpNavigationMarkers()
{
  resetUniqueId();

  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

  for (size_t i = 0; i < acummulated_poses_.size(); i++) {
    // Draw a green arrow at the waypoint pose
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header = acummulated_poses_[i].header;
    arrow_marker.id = getUniqueId();
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.pose = acummulated_poses_[i].pose;
    arrow_marker.scale.x = 0.3;
    arrow_marker.scale.y = 0.05;
    arrow_marker.scale.z = 0.02;
    arrow_marker.color.r = 0;
    arrow_marker.color.g = 255;
    arrow_marker.color.b = 0;
    arrow_marker.color.a = 1.0f;
    arrow_marker.lifetime = rclcpp::Duration(0);
    arrow_marker.frame_locked = false;
    marker_array->markers.push_back(arrow_marker);

    // Draw a red circle at the waypoint pose
    visualization_msgs::msg::Marker circle_marker;
    circle_marker.header = acummulated_poses_[i].header;
    circle_marker.id = getUniqueId();
    circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
    circle_marker.action = visualization_msgs::msg::Marker::ADD;
    circle_marker.pose = acummulated_poses_[i].pose;
    circle_marker.scale.x = 0.05;
    circle_marker.scale.y = 0.05;
    circle_marker.scale.z = 0.05;
    circle_marker.color.r = 255;
    circle_marker.color.g = 0;
    circle_marker.color.b = 0;
    circle_marker.color.a = 1.0f;
    circle_marker.lifetime = rclcpp::Duration(0);
    circle_marker.frame_locked = false;
    marker_array->markers.push_back(circle_marker);

    // Draw the waypoint number
    visualization_msgs::msg::Marker marker_text;
    marker_text.header = acummulated_poses_[i].header;
    marker_text.id = getUniqueId();
    marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::msg::Marker::ADD;
    marker_text.pose = acummulated_poses_[i].pose;
    marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
    marker_text.scale.x = 0.07;
    marker_text.scale.y = 0.07;
    marker_text.scale.z = 0.07;
    marker_text.color.r = 0;
    marker_text.color.g = 255;
    marker_text.color.b = 0;
    marker_text.color.a = 1.0f;
    marker_text.lifetime = rclcpp::Duration(0);
    marker_text.frame_locked = false;
    marker_text.text = "wp_" + std::to_string(i + 1);
    marker_array->markers.push_back(marker_text);
  }

  if (marker_array->markers.empty()) {
    visualization_msgs::msg::Marker clear_all_marker;
    clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(clear_all_marker);
  }

  wp_navigation_markers_pub_->publish(std::move(marker_array));
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Nav2Panel, rviz_common::Panel)
