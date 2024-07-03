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
#include <QHBoxLayout>
#include <QTextEdit>
#include <QCheckBox>

#include <ctype.h>
#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "nav2_rviz_plugins/goal_common.hpp"
#include "nav2_rviz_plugins/utils.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include "geometry_msgs/msg/pose2_d.hpp"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{
using nav2_util::geometry_utils::orientationAroundZAxis;

// Define global GoalPoseUpdater so that the nav2 GoalTool plugin can access to update goal pose
GoalPoseUpdater GoalUpdater;

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent),
  server_timeout_(100)
{
  // Create the control button and its tooltip

  start_reset_button_ = new QPushButton;
  pause_resume_button_ = new QPushButton;
  navigation_mode_button_ = new QPushButton;
  save_waypoints_button_ = new QPushButton;
  load_waypoints_button_ = new QPushButton;
  pause_waypoint_button_ = new QPushButton;
  navigation_status_indicator_ = new QLabel;
  localization_status_indicator_ = new QLabel;
  navigation_goal_status_indicator_ = new QLabel;
  navigation_feedback_indicator_ = new QLabel;
  waypoint_status_indicator_ = new QLabel;
  number_of_loops_ = new QLabel;
  nr_of_loops_ = new QLineEdit;
  store_initial_pose_checkbox_ = new QCheckBox("Store initial_pose");

  // Create the state machine used to present the proper control button states in the UI

  const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
  const char * shutdown_msg = "Deactivate and cleanup all nav2 lifecycle nodes";
  const char * cancel_msg = "Cancel navigation";
  const char * pause_msg = "Deactivate all nav2 lifecycle nodes";
  const char * resume_msg = "Activate all nav2 lifecycle nodes";
  const char * single_goal_msg = "Change to waypoint / nav through poses style navigation";
  const char * waypoint_goal_msg = "Start following waypoints";
  const char * nft_goal_msg = "Start navigating through poses";
  const char * cancel_waypoint_msg = "Cancel waypoint / viapoint accumulation mode";

  const QString navigation_active("<table><tr><td width=150><b>Navigation:</b></td>"
    "<td><font color=green>active</color></td></tr></table>");
  const QString navigation_inactive("<table><tr><td width=150><b>Navigation:</b></td>"
    "<td>inactive</td></tr></table>");
  const QString navigation_unknown("<table><tr><td width=150><b>Navigation:</b></td>"
    "<td>unknown</td></tr></table>");
  const QString localization_active("<table><tr><td width=150><b>Localization:</b></td>"
    "<td><font color=green>active</color></td></tr></table>");
  const QString localization_inactive("<table><tr><td width=150><b>Localization:</b></td>"
    "<td>inactive</td></tr></table>");
  const QString localization_unknown("<table><tr><td width=150><b>Localization:</b></td>"
    "<td>unknown</td></tr></table>");

  navigation_status_indicator_->setText(navigation_unknown);
  localization_status_indicator_->setText(localization_unknown);
  navigation_goal_status_indicator_->setText(nav2_rviz_plugins::getGoalStatusLabel());
  number_of_loops_->setText("Num of loops");
  navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
  navigation_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  localization_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  navigation_goal_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  navigation_feedback_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  waypoint_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  pre_initial_ = new QState();
  pre_initial_->setObjectName("pre_initial");
  pre_initial_->assignProperty(start_reset_button_, "text", "Startup");
  pre_initial_->assignProperty(start_reset_button_, "enabled", false);

  pre_initial_->assignProperty(pause_resume_button_, "text", "Pause");
  pre_initial_->assignProperty(pause_resume_button_, "enabled", false);

  pre_initial_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  pre_initial_->assignProperty(save_waypoints_button_, "enabled", false);

  pre_initial_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  pre_initial_->assignProperty(load_waypoints_button_, "enabled", false);

  pre_initial_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  pre_initial_->assignProperty(pause_waypoint_button_, "enabled", false);

  pre_initial_->assignProperty(nr_of_loops_, "text", "0");
  pre_initial_->assignProperty(nr_of_loops_, "enabled", false);

  pre_initial_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

  pre_initial_->assignProperty(
    navigation_mode_button_, "text",
    "Waypoint / Nav Through Poses Mode");
  pre_initial_->assignProperty(navigation_mode_button_, "enabled", false);

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(start_reset_button_, "text", "Startup");
  initial_->assignProperty(start_reset_button_, "toolTip", startup_msg);
  initial_->assignProperty(start_reset_button_, "enabled", true);

  initial_->assignProperty(pause_resume_button_, "text", "Pause");
  initial_->assignProperty(pause_resume_button_, "enabled", false);

  initial_->assignProperty(navigation_mode_button_, "text", "Waypoint / Nav Through Poses Mode");
  initial_->assignProperty(navigation_mode_button_, "enabled", false);

  initial_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  initial_->assignProperty(save_waypoints_button_, "enabled", false);

  initial_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  initial_->assignProperty(load_waypoints_button_, "enabled", false);

  initial_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  initial_->assignProperty(pause_waypoint_button_, "enabled", false);

  initial_->assignProperty(nr_of_loops_, "text", "0");
  initial_->assignProperty(nr_of_loops_, "enabled", false);

  initial_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

  // State entered when navigate_to_pose action is not active
  idle_ = new QState();
  idle_->setObjectName("idle");
  idle_->assignProperty(start_reset_button_, "text", "Reset");
  idle_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);
  idle_->assignProperty(start_reset_button_, "enabled", true);

  idle_->assignProperty(pause_resume_button_, "text", "Pause");
  idle_->assignProperty(pause_resume_button_, "enabled", true);
  idle_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

  idle_->assignProperty(navigation_mode_button_, "text", "Waypoint / Nav Through Poses Mode");
  idle_->assignProperty(navigation_mode_button_, "enabled", true);
  idle_->assignProperty(navigation_mode_button_, "toolTip", single_goal_msg);

  idle_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  idle_->assignProperty(save_waypoints_button_, "enabled", false);

  idle_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  idle_->assignProperty(load_waypoints_button_, "enabled", false);

  idle_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  idle_->assignProperty(pause_waypoint_button_, "enabled", false);

  idle_->assignProperty(nr_of_loops_, "text", "0");
  idle_->assignProperty(nr_of_loops_, "enabled", false);

  idle_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

  // State entered when navigate_to_pose action is not active
  accumulating_ = new QState();
  accumulating_->setObjectName("accumulating");
  accumulating_->assignProperty(start_reset_button_, "text", "Cancel Accumulation");
  accumulating_->assignProperty(start_reset_button_, "toolTip", cancel_waypoint_msg);
  accumulating_->assignProperty(start_reset_button_, "enabled", true);

  accumulating_->assignProperty(pause_resume_button_, "text", "Start Nav Through Poses");
  accumulating_->assignProperty(pause_resume_button_, "enabled", true);
  accumulating_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

  accumulating_->assignProperty(navigation_mode_button_, "text", "Start Waypoint Following");
  accumulating_->assignProperty(navigation_mode_button_, "enabled", true);
  accumulating_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

  accumulating_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  accumulating_->assignProperty(save_waypoints_button_, "enabled", true);

  accumulating_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  accumulating_->assignProperty(load_waypoints_button_, "enabled", true);

  accumulating_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  accumulating_->assignProperty(pause_waypoint_button_, "enabled", false);

  accumulating_->assignProperty(nr_of_loops_, "text", QString::fromStdString(loop_no_));
  accumulating_->assignProperty(nr_of_loops_, "enabled", true);
  accumulating_->assignProperty(store_initial_pose_checkbox_, "enabled", true);

  accumulated_wp_ = new QState();
  accumulated_wp_->setObjectName("accumulated_wp");
  accumulated_wp_->assignProperty(start_reset_button_, "text", "Cancel");
  accumulated_wp_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
  accumulated_wp_->assignProperty(start_reset_button_, "enabled", true);

  accumulated_wp_->assignProperty(pause_resume_button_, "text", "Start Nav Through Poses");
  accumulated_wp_->assignProperty(pause_resume_button_, "enabled", false);
  accumulated_wp_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

  accumulated_wp_->assignProperty(navigation_mode_button_, "text", "Start Waypoint Following");
  accumulated_wp_->assignProperty(navigation_mode_button_, "enabled", false);
  accumulated_wp_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

  accumulated_wp_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  accumulated_wp_->assignProperty(save_waypoints_button_, "enabled", false);

  accumulated_wp_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  accumulated_wp_->assignProperty(load_waypoints_button_, "enabled", false);

  accumulated_wp_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  accumulated_wp_->assignProperty(pause_waypoint_button_, "enabled", true);

  accumulated_wp_->assignProperty(nr_of_loops_, "enabled", false);
  accumulated_wp_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

  accumulated_nav_through_poses_ = new QState();
  accumulated_nav_through_poses_->setObjectName("accumulated_nav_through_poses");
  accumulated_nav_through_poses_->assignProperty(start_reset_button_, "text", "Cancel");
  accumulated_nav_through_poses_->assignProperty(start_reset_button_, "toolTip", cancel_msg);

  accumulated_nav_through_poses_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  accumulated_nav_through_poses_->assignProperty(save_waypoints_button_, "enabled", false);

  accumulated_nav_through_poses_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  accumulated_nav_through_poses_->assignProperty(load_waypoints_button_, "enabled", false);

  accumulated_nav_through_poses_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  accumulated_nav_through_poses_->assignProperty(pause_waypoint_button_, "enabled", false);

  accumulated_nav_through_poses_->assignProperty(nr_of_loops_, "enabled", false);

  accumulated_nav_through_poses_->assignProperty(start_reset_button_, "enabled", true);
  accumulated_nav_through_poses_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

  accumulated_nav_through_poses_->assignProperty(
    pause_resume_button_, "text",
    "Start Nav Through Poses");
  accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "enabled", false);
  accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

  accumulated_nav_through_poses_->assignProperty(
    navigation_mode_button_, "text",
    "Start Waypoint Following");
  accumulated_nav_through_poses_->assignProperty(navigation_mode_button_, "enabled", false);
  accumulated_nav_through_poses_->assignProperty(
    navigation_mode_button_, "toolTip",
    waypoint_goal_msg);

  accumulated_nav_through_poses_->assignProperty(
    nr_of_loops_, "text",
    QString::fromStdString(loop_no_));
  accumulated_nav_through_poses_->assignProperty(nr_of_loops_, "enabled", false);
  accumulated_nav_through_poses_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

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

  running_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  running_->assignProperty(save_waypoints_button_, "enabled", false);

  running_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  running_->assignProperty(load_waypoints_button_, "enabled", false);

  running_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  running_->assignProperty(pause_waypoint_button_, "enabled", false);

  running_->assignProperty(nr_of_loops_, "text", "0");
  running_->assignProperty(nr_of_loops_, "enabled", false);

  running_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

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

  paused_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  paused_->assignProperty(save_waypoints_button_, "enabled", false);

  paused_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  paused_->assignProperty(load_waypoints_button_, "enabled", false);

  paused_->assignProperty(pause_waypoint_button_, "text", "Pause WP");
  paused_->assignProperty(pause_waypoint_button_, "enabled", false);

  paused_->assignProperty(nr_of_loops_, "enabled", false);

  paused_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

  // State entered to resume the nav2 lifecycle nodes
  resumed_ = new QState();
  resumed_->setObjectName("resuming");

  // States entered to pause and Resume WPs
  resumed_wp_ = new QState();
  resumed_wp_->setObjectName("running");
  resumed_wp_->assignProperty(start_reset_button_, "text", "Cancel");
  resumed_wp_->assignProperty(start_reset_button_, "toolTip", cancel_msg);

  resumed_wp_->assignProperty(pause_resume_button_, "text", "Start Nav Through Poses");
  resumed_wp_->assignProperty(pause_resume_button_, "enabled", false);

  resumed_wp_->assignProperty(navigation_mode_button_, "text", "Start Waypoint Following");
  resumed_wp_->assignProperty(navigation_mode_button_, "enabled", false);

  resumed_wp_->assignProperty(save_waypoints_button_, "text", "Save WPs");
  resumed_wp_->assignProperty(save_waypoints_button_, "enabled", true);

  resumed_wp_->assignProperty(load_waypoints_button_, "text", "Load WPs");
  resumed_wp_->assignProperty(load_waypoints_button_, "enabled", false);

  resumed_wp_->assignProperty(pause_waypoint_button_, "text", "Resume WP");
  resumed_wp_->assignProperty(pause_waypoint_button_, "enabled", true);

  resumed_wp_->assignProperty(nr_of_loops_, "enabled", false);
  resumed_wp_->assignProperty(store_initial_pose_checkbox_, "enabled", false);

  QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
  QObject::connect(canceled_, SIGNAL(exited()), this, SLOT(onCancel()));
  QObject::connect(reset_, SIGNAL(exited()), this, SLOT(onShutdown()));
  QObject::connect(paused_, SIGNAL(entered()), this, SLOT(onPause()));
  QObject::connect(resumed_, SIGNAL(exited()), this, SLOT(onResume()));
  QObject::connect(accumulating_, SIGNAL(entered()), this, SLOT(onAccumulating()));
  QObject::connect(accumulated_wp_, SIGNAL(entered()), this, SLOT(onAccumulatedWp()));
  QObject::connect(resumed_wp_, SIGNAL(entered()), this, SLOT(onResumedWp()));
  QObject::connect(
    accumulated_nav_through_poses_, SIGNAL(entered()), this,
    SLOT(onAccumulatedNTP()));
  QObject::connect(
    save_waypoints_button_,
    &QPushButton::released,
    this, &Nav2Panel::handleGoalSaver);
  QObject::connect(
    load_waypoints_button_,
    &QPushButton::released,
    this,
    &Nav2Panel::handleGoalLoader);
  QObject::connect(
    nr_of_loops_,
    &QLineEdit::editingFinished,
    this,
    &Nav2Panel::loophandler);
  QObject::connect(
    store_initial_pose_checkbox_,
    &QCheckBox::stateChanged,
    this,
    &Nav2Panel::initialStateHandler);

  // Start/Reset button click transitions
  initial_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
  idle_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
  running_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
  paused_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
  idle_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulating_);
  accumulating_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulated_wp_);
  accumulating_->addTransition(
    pause_resume_button_, SIGNAL(
      clicked()), accumulated_nav_through_poses_);
  accumulating_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
  accumulated_wp_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
  accumulated_nav_through_poses_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);

  // Internal state transitions
  canceled_->addTransition(canceled_, SIGNAL(entered()), idle_);
  reset_->addTransition(reset_, SIGNAL(entered()), initial_);
  resumed_->addTransition(resumed_, SIGNAL(entered()), idle_);

  // Pause/Resume button click transitions
  idle_->addTransition(pause_resume_button_, SIGNAL(clicked()), paused_);
  paused_->addTransition(pause_resume_button_, SIGNAL(clicked()), resumed_);

  // Pause/Resume button waypoint transition
  accumulated_wp_->addTransition(pause_waypoint_button_, SIGNAL(clicked()), resumed_wp_);
  resumed_wp_->addTransition(pause_waypoint_button_, SIGNAL(clicked()), accumulated_wp_);
  resumed_wp_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);

  // ROSAction Transitions: So when actions are updated remotely (failing, succeeding, etc)
  // the state of the application will also update. This means that if in the processing
  // states and then goes inactive, move back to the idle state. Vise versa as well.
  ROSActionQTransition * idleTransition = new ROSActionQTransition(QActionState::INACTIVE);
  idleTransition->setTargetState(running_);
  idle_->addTransition(idleTransition);

  ROSActionQTransition * runningTransition = new ROSActionQTransition(QActionState::ACTIVE);
  runningTransition->setTargetState(idle_);
  running_->addTransition(runningTransition);

  ROSActionQTransition * idleAccumulatedWpTransition =
    new ROSActionQTransition(QActionState::INACTIVE);
  idleAccumulatedWpTransition->setTargetState(accumulated_wp_);
  idle_->addTransition(idleAccumulatedWpTransition);

  ROSActionQTransition * accumulatedWpTransition = new ROSActionQTransition(QActionState::ACTIVE);
  accumulatedWpTransition->setTargetState(accumulating_);
  accumulated_wp_->addTransition(accumulatedWpTransition);

  ROSActionQTransition * idleAccumulatedNTPTransition =
    new ROSActionQTransition(QActionState::INACTIVE);
  idleAccumulatedNTPTransition->setTargetState(accumulated_nav_through_poses_);
  idle_->addTransition(idleAccumulatedNTPTransition);

  ROSActionQTransition * accumulatedNTPTransition = new ROSActionQTransition(QActionState::ACTIVE);
  accumulatedNTPTransition->setTargetState(idle_);
  accumulated_nav_through_poses_->addTransition(accumulatedNTPTransition);

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "--remap", "__node:=rviz_navigation_dialog_action_client", "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  client_nav_ = std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_navigation", client_node_);
  client_loc_ = std::make_shared<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_localization", client_node_);
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
      navigation_goal_status_indicator_->setText(nav2_rviz_plugins::getGoalStatusLabel());
      navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
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
  state_machine_.addState(accumulated_wp_);
  state_machine_.addState(accumulated_nav_through_poses_);
  state_machine_.addState(resumed_wp_);

  state_machine_.setInitialState(pre_initial_);

  // delay starting initial thread until state machine has started or a race occurs
  QObject::connect(&state_machine_, SIGNAL(started()), this, SLOT(startThread()));
  state_machine_.start();

  // Lay out the items in the panel
  QVBoxLayout * main_layout = new QVBoxLayout;
  QHBoxLayout * side_layout = new QHBoxLayout;
  QVBoxLayout * status_layout = new QVBoxLayout;
  QHBoxLayout * logo_layout = new QHBoxLayout;
  QVBoxLayout * group_box_layout = new QVBoxLayout;

  QGroupBox * groupBox = new QGroupBox(tr("Tools for WP-Following"));
  imgDisplayLabel_ = new QLabel("");
  imgDisplayLabel_->setPixmap(
    rviz_common::loadPixmap("package://nav2_rviz_plugins/icons/classes/nav2_logo_small.png"));

  status_layout->addWidget(navigation_status_indicator_);
  status_layout->addWidget(localization_status_indicator_);
  status_layout->addWidget(navigation_goal_status_indicator_);

  logo_layout->addWidget(imgDisplayLabel_, 5, Qt::AlignRight);

  side_layout->addLayout(status_layout);
  side_layout->addLayout(logo_layout);

  main_layout->addLayout(side_layout);
  main_layout->addWidget(navigation_feedback_indicator_);
  main_layout->addWidget(waypoint_status_indicator_);
  main_layout->addWidget(pause_resume_button_);
  main_layout->addWidget(start_reset_button_);
  main_layout->addWidget(navigation_mode_button_);

  QHBoxLayout * group_box_l1_layout = new QHBoxLayout;
  QHBoxLayout * group_box_l2_layout = new QHBoxLayout;

  group_box_l1_layout->addWidget(save_waypoints_button_);
  group_box_l1_layout->addWidget(load_waypoints_button_);
  group_box_l1_layout->addWidget(pause_waypoint_button_);

  group_box_l2_layout->addWidget(number_of_loops_);
  group_box_l2_layout->addWidget(nr_of_loops_);
  group_box_l2_layout->addWidget(store_initial_pose_checkbox_);

  group_box_layout->addLayout(group_box_l1_layout);
  group_box_layout->addLayout(group_box_l2_layout);

  groupBox->setLayout(group_box_layout);

  main_layout->addWidget(groupBox);
  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);

  navigation_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_,
    "navigate_to_pose");
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    client_node_,
    "follow_waypoints");
  nav_through_poses_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    client_node_,
    "navigate_through_poses");

  // Setting up tf for initial pose
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(client_node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    client_node_->get_node_base_interface(),
    client_node_->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
  waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();
  nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();

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

void Nav2Panel::initialStateHandler()
{
  if (store_initial_pose_checkbox_->isChecked()) {
    store_initial_pose_ = true;

  } else {
    store_initial_pose_ = false;
  }
}

bool Nav2Panel::isLoopValueValid(std::string & loop_value)
{
  // Check for just empty space
  if (loop_value.empty()) {
    std::cout << "Loop value cannot be set to empty, setting to 0" << std::endl;
    loop_value = "0";
    nr_of_loops_->setText("0");
    return true;
  }

  // Check for any chars or spaces in the string
  for (char & c : loop_value) {
    if (isalpha(c) || isspace(c) || ispunct(c)) {
      waypoint_status_indicator_->setText(
        "<b> Note: </b> Set a valid value for the loop");
      std::cout << "Set a valid value for the loop, check for alphabets and spaces" << std::endl;
      navigation_mode_button_->setEnabled(false);
      return false;
    }
  }

  try {
    stoi(loop_value);
  } catch (std::invalid_argument const & ex) {
    // Handling special symbols
    waypoint_status_indicator_->setText("<b> Note: </b> Set a valid value for the loop");
    navigation_mode_button_->setEnabled(false);
    return false;
  } catch (std::out_of_range const & ex) {
    // Handling out of range values
    waypoint_status_indicator_->setText(
      "<b> Note: </b> Loop value out of range, setting max possible value"
    );
    loop_value = std::to_string(std::numeric_limits<int>::max());
    nr_of_loops_->setText(QString::fromStdString(loop_value));
  }
  return true;
}

void Nav2Panel::loophandler()
{
  loop_no_ = nr_of_loops_->displayText().toStdString();

  // Sanity check for the loop value
  if (!isLoopValueValid(loop_no_)) {
    return;
  }

  // Enabling the start_waypoint_follow button, if disabled.
  navigation_mode_button_->setEnabled(true);

  // Disabling nav_through_poses button
  if (!loop_no_.empty() && stoi(loop_no_) > 0) {
    pause_resume_button_->setEnabled(false);
  } else {
    pause_resume_button_->setEnabled(true);
  }
}

void Nav2Panel::handleGoalLoader()
{
  acummulated_poses_.clear();

  std::cout << "Loading Waypoints!" << std::endl;

  QString file = QFileDialog::getOpenFileName(
    this,
    tr("Open File"), "",
    tr("yaml(*.yaml);;All Files (*)"));

  YAML::Node available_waypoints;

  try {
    available_waypoints = YAML::LoadFile(file.toStdString());
  } catch (const std::exception & ex) {
    std::cout << ex.what() << ", please select a valid file" << std::endl;
    updateWpNavigationMarkers();
    return;
  }

  const YAML::Node & waypoint_iter = available_waypoints["waypoints"];
  for (YAML::const_iterator it = waypoint_iter.begin(); it != waypoint_iter.end(); ++it) {
    auto waypoint = waypoint_iter[it->first.as<std::string>()];
    auto pose = waypoint["pose"].as<std::vector<double>>();
    auto orientation = waypoint["orientation"].as<std::vector<double>>();
    acummulated_poses_.push_back(convert_to_msg(pose, orientation));
  }

  // Publishing Waypoint Navigation marker after loading wp's
  updateWpNavigationMarkers();
}

geometry_msgs::msg::PoseStamped Nav2Panel::convert_to_msg(
  std::vector<double> pose,
  std::vector<double> orientation)
{
  auto msg = geometry_msgs::msg::PoseStamped();

  msg.header.frame_id = "map";
  // msg.header.stamp = client_node_->now();  // client node doesn't respect sim time yet

  msg.pose.position.x = pose[0];
  msg.pose.position.y = pose[1];
  msg.pose.position.z = pose[2];

  msg.pose.orientation.w = orientation[0];
  msg.pose.orientation.x = orientation[1];
  msg.pose.orientation.y = orientation[2];
  msg.pose.orientation.z = orientation[3];

  return msg;
}

void Nav2Panel::handleGoalSaver()
{
// Check if the waypoints are accumulated

  if (acummulated_poses_.empty()) {
    std::cout << "No accumulated Points to Save!" << std::endl;
    return;
  } else {
    std::cout << "Saving Waypoints!" << std::endl;
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "waypoints";
  out << YAML::BeginMap;

  // Save WPs to data structure
  for (unsigned int i = 0; i < acummulated_poses_.size(); ++i) {
    out << YAML::Key << "waypoint" + std::to_string(i);
    out << YAML::BeginMap;
    out << YAML::Key << "pose";
    std::vector<double> pose =
    {acummulated_poses_[i].pose.position.x, acummulated_poses_[i].pose.position.y,
      acummulated_poses_[i].pose.position.z};
    out << YAML::Value << pose;
    out << YAML::Key << "orientation";
    std::vector<double> orientation =
    {acummulated_poses_[i].pose.orientation.w, acummulated_poses_[i].pose.orientation.x,
      acummulated_poses_[i].pose.orientation.y, acummulated_poses_[i].pose.orientation.z};
    out << YAML::Value << orientation;
    out << YAML::EndMap;
  }

  // open dialog to save it in a file
  QString file = QFileDialog::getSaveFileName(
    this,
    tr("Open File"), "",
    tr("yaml(*.yaml);;All Files (*)"));

  if (!file.toStdString().empty()) {
    std::ofstream fout(file.toStdString() + ".yaml");
    fout << out.c_str();
    std::cout << "Saving waypoints succeeded" << std::endl;
  } else {
    std::cout << "Saving waypoints aborted" << std::endl;
  }
}

void
Nav2Panel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // declaring parameter to get the base frame
  node->declare_parameter("base_frame", rclcpp::ParameterValue(std::string("base_footprint")));
  node->get_parameter("base_frame", base_frame_);

  // create action feedback subscribers
  navigation_feedback_sub_ =
    node->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
    "navigate_to_pose/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg) {
      if (stoi(nr_of_loops_->displayText().toStdString()) > 0) {
        if (goal_index_ == 0 && !loop_counter_stop_) {
          loop_count_++;
          loop_counter_stop_ = true;
        }
        if (goal_index_ != 0) {
          loop_counter_stop_ = false;
        }
        navigation_feedback_indicator_->setText(
          getNavToPoseFeedbackLabel(msg->feedback) + QString(
            std::string(
              "</td></tr><tr><td width=150>Waypoint:</td><td>" +
              toString(goal_index_ + 1)).c_str()) + QString(
            std::string(
              "</td></tr><tr><td width=150>Loop:</td><td>" +
              toString(loop_count_)).c_str()));
      } else {
        navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel(msg->feedback));
      }
    });
  nav_through_poses_feedback_sub_ =
    node->create_subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>(
    "navigate_through_poses/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg) {
      navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel(msg->feedback));
    });

  // create action goal status subscribers
  navigation_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      navigation_goal_status_indicator_->setText(
        nav2_rviz_plugins::getGoalStatusLabel("Feedback", msg->status_list.back().status));
      // Clearing all the stored values once reaching the final goal
      if (
        loop_count_ == stoi(nr_of_loops_->displayText().toStdString()) &&
        goal_index_ == static_cast<int>(store_poses_.size()) - 1 &&
        msg->status_list.back().status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        store_poses_.clear();
        waypoint_status_indicator_->clear();
        loop_no_ = "0";
        loop_count_ = 0;
        navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel());
      }
    });
  nav_through_poses_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_through_poses/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
      navigation_goal_status_indicator_->setText(
        nav2_rviz_plugins::getGoalStatusLabel("Feedback", msg->status_list.back().status));
      if (msg->status_list.back().status != action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
      }
    });
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
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::pause,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onResume()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::resume,
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::resume,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onStartup()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::startup,
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::startup,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
}

void
Nav2Panel::onShutdown()
{
  QFuture<void> futureNav =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::reset,
      client_nav_.get(), std::placeholders::_1), server_timeout_);
  QFuture<void> futureLoc =
    QtConcurrent::run(
    std::bind(
      &nav2_lifecycle_manager::LifecycleManagerClient::reset,
      client_loc_.get(), std::placeholders::_1), server_timeout_);
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
  waypoint_status_indicator_->clear();
  store_poses_.clear();
  acummulated_poses_.clear();
}

void Nav2Panel::onResumedWp()
{
  QFuture<void> future =
    QtConcurrent::run(
    std::bind(
      &Nav2Panel::onCancelButtonPressed,
      this));
  acummulated_poses_ = store_poses_;
  loop_no_ = std::to_string(
    stoi(nr_of_loops_->displayText().toStdString()) -
    loop_count_);
  waypoint_status_indicator_->setText(
    QString(std::string("<b> Note: </b> Navigation is paused.").c_str()));
}

void
Nav2Panel::onNewGoal(double x, double y, double theta, QString frame)
{
  auto pose = geometry_msgs::msg::PoseStamped();

  // pose.header.stamp = client_node_->now();  // client node doesn't respect sim time yet
  pose.header.frame_id = frame.toStdString();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = orientationAroundZAxis(theta);

  if (store_poses_.empty()) {
    if (state_machine_.configuration().contains(accumulating_)) {
      waypoint_status_indicator_->clear();
      acummulated_poses_.push_back(pose);
    } else {
      acummulated_poses_.clear();
      updateWpNavigationMarkers();
      std::cout << "Start navigation" << std::endl;
      startNavigation(pose);
    }
  } else {
    waypoint_status_indicator_->setText(
      QString(std::string("<b> Note: </b> Cannot set goal in pause state").c_str()));
  }
  updateWpNavigationMarkers();
}

void
Nav2Panel::onCancelButtonPressed()
{
  if (navigation_goal_handle_) {
    auto future_cancel = navigation_action_client_->async_cancel_goal(navigation_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    } else {
      navigation_goal_handle_.reset();
    }
  }

  if (waypoint_follower_goal_handle_) {
    auto future_cancel =
      waypoint_follower_action_client_->async_cancel_goal(waypoint_follower_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel waypoint follower");
    } else {
      waypoint_follower_goal_handle_.reset();
    }
  }

  if (nav_through_poses_goal_handle_) {
    auto future_cancel =
      nav_through_poses_action_client_->async_cancel_goal(nav_through_poses_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel nav through pose action");
    } else {
      nav_through_poses_goal_handle_.reset();
    }
  }

  timer_.stop();
}

void
Nav2Panel::onAccumulatedWp()
{
  if (acummulated_poses_.empty()) {
    state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
    waypoint_status_indicator_->setText(
      "<b> Note: </b> Uh oh! Someone forgot to select the waypoints");
    return;
  }

  // Sanity check for the loop value
  if (!isLoopValueValid(loop_no_)) {
    state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
    return;
  }

  // Remove any warning status at this point
  waypoint_status_indicator_->clear();

  // Disable navigation modes
  navigation_mode_button_->setEnabled(false);
  pause_resume_button_->setEnabled(false);

  /** Making sure that the pose array does not get updated
   *  between the process**/
  if (store_poses_.empty()) {
    std::cout << "Start waypoint" << std::endl;

    // Setting the final loop value on the text box for sanity
    nr_of_loops_->setText(QString::fromStdString(loop_no_));

    // Variable to store initial pose
    geometry_msgs::msg::TransformStamped init_transform;

    // Looking up transform to get initial pose
    if (store_initial_pose_) {
      try {
        init_transform = tf2_buffer_->lookupTransform(
          acummulated_poses_[0].header.frame_id, base_frame_,
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          client_node_->get_logger(), "Could not transform %s to %s: %s",
          acummulated_poses_[0].header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        return;
      }

      // Converting TransformStamped to PoseStamped
      geometry_msgs::msg::PoseStamped initial_pose;
      initial_pose.header = init_transform.header;
      initial_pose.pose.position.x = init_transform.transform.translation.x;
      initial_pose.pose.position.y = init_transform.transform.translation.y;
      initial_pose.pose.position.z = init_transform.transform.translation.z;
      initial_pose.pose.orientation.x = init_transform.transform.rotation.x;
      initial_pose.pose.orientation.y = init_transform.transform.rotation.y;
      initial_pose.pose.orientation.z = init_transform.transform.rotation.z;
      initial_pose.pose.orientation.w = init_transform.transform.rotation.w;

      // inserting the acummulated pose
      acummulated_poses_.insert(acummulated_poses_.begin(), initial_pose);
      updateWpNavigationMarkers();
      initial_pose_stored_ = true;
      if (loop_count_ == 0) {
        goal_index_ = 1;
      }
    } else if (!store_initial_pose_ && initial_pose_stored_) {
      acummulated_poses_.erase(
        acummulated_poses_.begin(),
        acummulated_poses_.begin());
    }
  } else {
    std::cout << "Resuming waypoint" << std::endl;
  }

  startWaypointFollowing(acummulated_poses_);
  store_poses_ = acummulated_poses_;
  acummulated_poses_.clear();
}

void
Nav2Panel::onAccumulatedNTP()
{
  std::cout << "Start navigate through poses" << std::endl;
  startNavThroughPoses(acummulated_poses_);
}

void
Nav2Panel::onAccumulating()
{
  acummulated_poses_.clear();
  store_poses_.clear();
  loop_count_ = 0;
  loop_no_ = "0";
  initial_pose_stored_ = false;
  loop_counter_stop_ = true;
  goal_index_ = 0;
  updateWpNavigationMarkers();
}

void
Nav2Panel::timerEvent(QTimerEvent * event)
{
  if (state_machine_.configuration().contains(accumulated_wp_)) {
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
  } else if (state_machine_.configuration().contains(accumulated_nav_through_poses_)) {
    if (event->timerId() == timer_.timerId()) {
      if (!nav_through_poses_goal_handle_) {
        RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
        state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
        return;
      }

      rclcpp::spin_some(client_node_);
      auto status = nav_through_poses_goal_handle_->get_status();

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
      client_node_->get_logger(), "follow_waypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;
  waypoint_follower_goal_.goal_index = goal_index_;
  waypoint_follower_goal_.number_of_loops = stoi(loop_no_);

  RCLCPP_DEBUG(
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
  send_goal_options.result_callback = [this](auto) {
      waypoint_follower_goal_handle_.reset();
    };

  send_goal_options.feedback_callback = [this](
    WaypointFollowerGoalHandle::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback) {
      goal_index_ = feedback->current_waypoint;
    };

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
Nav2Panel::startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(), "navigate_through_poses action server is not available."
      " Is the initial pose set?");
    return;
  }

  nav_through_poses_goal_.poses = poses;
  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateThroughPoses will be called using the BT Navigator's default behavior tree.");

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    nav_through_poses_goal_.poses.size());
  for (auto waypoint : nav_through_poses_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      nav_through_poses_goal_handle_.reset();
    };

  auto future_goal_handle =
    nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  nav_through_poses_goal_handle_ = future_goal_handle.get();
  if (!nav_through_poses_goal_handle_) {
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
      "navigate_to_pose action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal pose
  navigation_goal_.pose = pose;

  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      navigation_goal_handle_.reset();
    };

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
    arrow_marker.lifetime = rclcpp::Duration(0s);
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
    circle_marker.lifetime = rclcpp::Duration(0s);
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
    marker_text.lifetime = rclcpp::Duration(0s);
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

inline QString
Nav2Panel::getNavToPoseFeedbackLabel(nav2_msgs::action::NavigateToPose::Feedback msg)
{
  return QString(std::string("<table>" + toLabel(msg) + "</table>").c_str());
}

inline QString
Nav2Panel::getNavThroughPosesFeedbackLabel(nav2_msgs::action::NavigateThroughPoses::Feedback msg)
{
  return QString(
    std::string(
      "<table><tr><td width=150>Poses remaining:</td><td>" +
      std::to_string(msg.number_of_poses_remaining) +
      "</td></tr>" + toLabel(msg) + "</table>").c_str());
}

template<typename T>
inline std::string Nav2Panel::toLabel(T & msg)
{
  return std::string(
    "<tr><td width=150>ETA:</td><td>" +
    toString(rclcpp::Duration(msg.estimated_time_remaining).seconds(), 0) + " s"
    "</td></tr><tr><td width=150>Distance remaining:</td><td>" +
    toString(msg.distance_remaining, 2) + " m"
    "</td></tr><tr><td width=150>Time taken:</td><td>" +
    toString(rclcpp::Duration(msg.navigation_time).seconds(), 0) + " s"
    "</td></tr><tr><td width=150>Recoveries:</td><td>" +
    std::to_string(msg.number_of_recoveries) +
    "</td></tr>");
}

inline std::string
Nav2Panel::toString(double val, int precision)
{
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << val;
  return out.str();
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Nav2Panel, rviz_common::Panel)
