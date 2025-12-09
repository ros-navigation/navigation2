// Copyright (c) Pau Reverté 2025
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

#include "nav2_rviz_plugins/navigate_through_poses_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QMessageBox>
#include <QTimer>

#include <memory>
#include <chrono>
#include <cmath>

#include "rviz_common/display_context.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;

namespace nav2_rviz_plugins
{

NavigateThroughPosesPanel::NavigateThroughPosesPanel(QWidget * parent)
: rviz_common::Panel(parent),
  current_action_name_("navigate_through_poses"),
  goal_active_(false)
{
  setupUI();
}

NavigateThroughPosesPanel::~NavigateThroughPosesPanel()
{
}

void NavigateThroughPosesPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node) {
    return;
  }

  node_ptr_ = node;

  if (node_ptr_) {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      node_ptr_->get_raw_node(), current_action_name_);

    QTimer * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [this, timer]() {
        if (action_client_ && action_client_->wait_for_action_server(std::chrono::seconds(0))) {
          if (send_goal_button_) {
            send_goal_button_->setEnabled(true);
          }
          timer->stop();
          timer->deleteLater();
        }
    });
    timer->start(1000);
  }
}

void NavigateThroughPosesPanel::setupUI()
{
  QVBoxLayout * main_layout = new QVBoxLayout;

  QGroupBox * action_group = new QGroupBox("Action Configuration");
  QGridLayout * action_layout = new QGridLayout;

  action_layout->addWidget(new QLabel("Action Name:"), 0, 0);
  action_name_edit_ = new QLineEdit("navigate_through_poses");
  connect(action_name_edit_, &QLineEdit::textChanged,
    this, &NavigateThroughPosesPanel::onActionNameChanged);
  action_layout->addWidget(action_name_edit_, 0, 1);

  action_group->setLayout(action_layout);
  main_layout->addWidget(action_group);

  QGroupBox * poses_group = new QGroupBox("Poses Configuration");
  QVBoxLayout * poses_layout = new QVBoxLayout;

  poses_tab_widget_ = new QTabWidget();
  poses_layout->addWidget(poses_tab_widget_);

  QHBoxLayout * pose_buttons_layout = new QHBoxLayout;

  add_pose_button_ = new QPushButton("Add Pose");
  connect(add_pose_button_, &QPushButton::clicked, this, &NavigateThroughPosesPanel::onAddPose);
  pose_buttons_layout->addWidget(add_pose_button_);

  remove_pose_button_ = new QPushButton("Remove Pose");
  connect(remove_pose_button_, &QPushButton::clicked,
    this, &NavigateThroughPosesPanel::onRemovePose);
  pose_buttons_layout->addWidget(remove_pose_button_);

  pose_buttons_layout->addStretch();
  poses_layout->addLayout(pose_buttons_layout);

  poses_group->setLayout(poses_layout);
  main_layout->addWidget(poses_group);

  createPoseTab(0);

  QGroupBox * bt_group = new QGroupBox("Behavior Tree");
  QVBoxLayout * bt_layout = new QVBoxLayout;

  bt_layout->addWidget(new QLabel("Behavior Tree XML path:"));
  behavior_tree_edit_ = new QLineEdit();
  behavior_tree_edit_->setPlaceholderText("Leave empty for default behavior tree");
  bt_layout->addWidget(behavior_tree_edit_);

  bt_group->setLayout(bt_layout);
  main_layout->addWidget(bt_group);

  QHBoxLayout * button_layout = new QHBoxLayout;

  send_goal_button_ = new QPushButton("Send Goal");
  send_goal_button_->setEnabled(false);
  connect(send_goal_button_, &QPushButton::clicked, this, &NavigateThroughPosesPanel::onSendGoal);
  button_layout->addWidget(send_goal_button_);

  cancel_button_ = new QPushButton("Cancel Goal");
  cancel_button_->setEnabled(false);
  connect(cancel_button_, &QPushButton::clicked, this, &NavigateThroughPosesPanel::onCancel);
  button_layout->addWidget(cancel_button_);

  main_layout->addLayout(button_layout);

  setLayout(main_layout);
}

void NavigateThroughPosesPanel::createPoseTab(int index)
{
  PoseTab pose_tab;

  pose_tab.widget = new QWidget();
  QGridLayout * pose_layout = new QGridLayout;

  pose_layout->addWidget(new QLabel("Frame ID:"), 0, 0);
  pose_tab.frame_id_edit = new QLineEdit("map");
  pose_layout->addWidget(pose_tab.frame_id_edit, 0, 1);

  pose_layout->addWidget(new QLabel("Position X (m):"), 1, 0);
  pose_tab.pos_x_spin = new QDoubleSpinBox();
  pose_tab.pos_x_spin->setRange(-1000.0, 1000.0);
  pose_tab.pos_x_spin->setDecimals(3);
  pose_tab.pos_x_spin->setSingleStep(0.1);
  pose_layout->addWidget(pose_tab.pos_x_spin, 1, 1);

  pose_layout->addWidget(new QLabel("Position Y (m):"), 2, 0);
  pose_tab.pos_y_spin = new QDoubleSpinBox();
  pose_tab.pos_y_spin->setRange(-1000.0, 1000.0);
  pose_tab.pos_y_spin->setDecimals(3);
  pose_tab.pos_y_spin->setSingleStep(0.1);
  pose_layout->addWidget(pose_tab.pos_y_spin, 2, 1);

  pose_layout->addWidget(new QLabel("Position Z (m):"), 3, 0);
  pose_tab.pos_z_spin = new QDoubleSpinBox();
  pose_tab.pos_z_spin->setRange(-1000.0, 1000.0);
  pose_tab.pos_z_spin->setDecimals(3);
  pose_tab.pos_z_spin->setSingleStep(0.1);
  pose_tab.pos_z_spin->setValue(0.0);
  pose_layout->addWidget(pose_tab.pos_z_spin, 3, 1);

  pose_layout->addWidget(new QLabel("Yaw (radians):"), 4, 0);
  pose_tab.yaw_spin = new QDoubleSpinBox();
  pose_tab.yaw_spin->setRange(-M_PI, M_PI);
  pose_tab.yaw_spin->setDecimals(4);
  pose_tab.yaw_spin->setSingleStep(0.01);
  pose_layout->addWidget(pose_tab.yaw_spin, 4, 1);

  pose_tab.widget->setLayout(pose_layout);

  QString tab_name = QString("Pose %1").arg(index + 1);
  poses_tab_widget_->addTab(pose_tab.widget, tab_name);

  pose_tabs_.push_back(pose_tab);

  remove_pose_button_->setEnabled(pose_tabs_.size() > 1);
}

void NavigateThroughPosesPanel::onAddPose()
{
  createPoseTab(pose_tabs_.size());
}

void NavigateThroughPosesPanel::onRemovePose()
{
  if (pose_tabs_.size() <= 1) {
    return;
  }

  int current_index = poses_tab_widget_->currentIndex();
  if (current_index >= 0 && current_index < static_cast<int>(pose_tabs_.size())) {
    poses_tab_widget_->removeTab(current_index);

    pose_tabs_.erase(pose_tabs_.begin() + current_index);

    for (int i = 0; i < static_cast<int>(pose_tabs_.size()); ++i) {
      QString tab_name = QString("Pose %1").arg(i + 1);
      poses_tab_widget_->setTabText(i, tab_name);
    }

    remove_pose_button_->setEnabled(pose_tabs_.size() > 1);
  }
}

void NavigateThroughPosesPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString action_name;
  if (config.mapGetString("action_name", &action_name) && action_name_edit_) {
    action_name_edit_->setText(action_name);
    current_action_name_ = action_name.toStdString();
  }
}

void NavigateThroughPosesPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  if (action_name_edit_) {
    config.mapSetValue("action_name", action_name_edit_->text());
  }
}

void NavigateThroughPosesPanel::onSendGoal()
{
  if (!action_client_) {
    return;
  }

  if (!behavior_tree_edit_) {
    return;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();

  goal_msg.poses = getCurrentPoses();

  goal_msg.behavior_tree = behavior_tree_edit_->text().toStdString();

  auto send_goal_options = rclcpp_action::Client
    <nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle
    <nav2_msgs::action::NavigateThroughPoses>::SharedPtr & goal_handle) {
      if (!goal_handle) {
        goal_active_ = false;
        if (send_goal_button_) {
          send_goal_button_->setEnabled(true);
        }
        if (cancel_button_) {
          cancel_button_->setEnabled(false);
        }
      } else {
        goal_handle_ = goal_handle;
        goal_active_ = true;
        if (send_goal_button_) {
          send_goal_button_->setEnabled(false);
        }
        if (cancel_button_) {
          cancel_button_->setEnabled(true);
        }
      }
    };

  send_goal_options.result_callback =
    std::bind(&NavigateThroughPosesPanel::onActionResult, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

  if (send_goal_button_) {
    send_goal_button_->setEnabled(false);
  }
}

void NavigateThroughPosesPanel::onCancel()
{
  if (goal_handle_ && goal_active_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
}

void NavigateThroughPosesPanel::onActionNameChanged()
{
  if (!action_name_edit_) {
    return;
  }

  QString new_name = action_name_edit_->text();
  if (new_name.isEmpty()) {
    return;
  }

  current_action_name_ = new_name.toStdString();

  if (node_ptr_) {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      node_ptr_->get_raw_node(), current_action_name_);

    if (send_goal_button_) {
      send_goal_button_->setEnabled(false);
    }

    QTimer::singleShot(100, [this]() {
        if (action_client_ && action_client_->wait_for_action_server(std::chrono::seconds(0))) {
          if (send_goal_button_) {
            send_goal_button_->setEnabled(true);
          }
        }
    });
  }
}

nav_msgs::msg::Goals NavigateThroughPosesPanel::getCurrentPoses()
{
  nav_msgs::msg::Goals goals;

  if (!node_ptr_) {
    return goals;
  }

  for (const auto & pose_tab : pose_tabs_) {
    if (pose_tab.frame_id_edit && pose_tab.pos_x_spin && pose_tab.pos_y_spin &&
      pose_tab.pos_z_spin && pose_tab.yaw_spin)
    {
      geometry_msgs::msg::PoseStamped pose = getPoseFromTab(pose_tab);
      goals.goals.push_back(pose);
    }
  }

  return goals;
}

geometry_msgs::msg::PoseStamped NavigateThroughPosesPanel::getPoseFromTab(const PoseTab & tab)
{
  geometry_msgs::msg::PoseStamped pose;

  if (!node_ptr_) {
    return pose;
  }

  pose.header.stamp = node_ptr_->get_raw_node()->now();
  pose.header.frame_id = tab.frame_id_edit->text().toStdString();

  pose.pose.position.x = tab.pos_x_spin->value();
  pose.pose.position.y = tab.pos_y_spin->value();
  pose.pose.position.z = tab.pos_z_spin->value();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, tab.yaw_spin->value());
  pose.pose.orientation.x = q.getX();
  pose.pose.orientation.y = q.getY();
  pose.pose.orientation.z = q.getZ();
  pose.pose.orientation.w = q.getW();

  return pose;
}

void NavigateThroughPosesPanel::onActionResult(
  const rclcpp_action::ClientGoalHandle
  <nav2_msgs::action::NavigateThroughPoses>::WrappedResult & /* result */)
{
  goal_active_ = false;
  if (send_goal_button_) {
    send_goal_button_->setEnabled(true);
  }
  if (cancel_button_) {
    cancel_button_->setEnabled(false);
  }
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::NavigateThroughPosesPanel, rviz_common::Panel)
