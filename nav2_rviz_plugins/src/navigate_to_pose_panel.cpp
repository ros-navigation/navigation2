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

#include "nav2_rviz_plugins/navigate_to_pose_panel.hpp"

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

NavigateToPosePanel::NavigateToPosePanel(QWidget * parent)
: rviz_common::Panel(parent),
  current_action_name_("navigate_to_pose"),
  goal_active_(false)
{
  setupUI();
}

NavigateToPosePanel::~NavigateToPosePanel()
{
}

void NavigateToPosePanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node) {
    return;
  }
  
  node_ptr_ = node;
  
  if (node_ptr_) {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
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

void NavigateToPosePanel::setupUI()
{
  QVBoxLayout * main_layout = new QVBoxLayout;
  
  QGroupBox * action_group = new QGroupBox("Action Configuration");
  QGridLayout * action_layout = new QGridLayout;
  
  action_layout->addWidget(new QLabel("Action Name:"), 0, 0);
  action_name_edit_ = new QLineEdit("navigate_to_pose");
  connect(action_name_edit_, &QLineEdit::textChanged, this, &NavigateToPosePanel::onActionNameChanged);
  action_layout->addWidget(action_name_edit_, 0, 1);
  
  action_group->setLayout(action_layout);
  main_layout->addWidget(action_group);
  
  QGroupBox * pose_group = new QGroupBox("Pose Configuration");
  QGridLayout * pose_layout = new QGridLayout;
  
  pose_layout->addWidget(new QLabel("Frame ID:"), 0, 0);
  frame_id_edit_ = new QLineEdit("map");
  pose_layout->addWidget(frame_id_edit_, 0, 1);
  
  pose_layout->addWidget(new QLabel("Position X (m):"), 1, 0);
  pos_x_spin_ = new QDoubleSpinBox();
  pos_x_spin_->setRange(-1000.0, 1000.0);
  pos_x_spin_->setDecimals(3);
  pos_x_spin_->setSingleStep(0.1);
  pose_layout->addWidget(pos_x_spin_, 1, 1);
  
  pose_layout->addWidget(new QLabel("Position Y (m):"), 2, 0);
  pos_y_spin_ = new QDoubleSpinBox();
  pos_y_spin_->setRange(-1000.0, 1000.0);
  pos_y_spin_->setDecimals(3);
  pos_y_spin_->setSingleStep(0.1);
  pose_layout->addWidget(pos_y_spin_, 2, 1);
  
  pose_layout->addWidget(new QLabel("Position Z (m):"), 3, 0);
  pos_z_spin_ = new QDoubleSpinBox();
  pos_z_spin_->setRange(-1000.0, 1000.0);
  pos_z_spin_->setDecimals(3);
  pos_z_spin_->setSingleStep(0.1);
  pos_z_spin_->setValue(0.0);
  pose_layout->addWidget(pos_z_spin_, 3, 1);
  
  pose_layout->addWidget(new QLabel("Yaw (radians):"), 4, 0);
  yaw_spin_ = new QDoubleSpinBox();
  yaw_spin_->setRange(-M_PI, M_PI);
  yaw_spin_->setDecimals(4);
  yaw_spin_->setSingleStep(0.01);
  pose_layout->addWidget(yaw_spin_, 4, 1);
  
  pose_group->setLayout(pose_layout);
  main_layout->addWidget(pose_group);

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
  connect(send_goal_button_, &QPushButton::clicked, this, &NavigateToPosePanel::onSendGoal);
  button_layout->addWidget(send_goal_button_);
  
  cancel_button_ = new QPushButton("Cancel Goal");
  cancel_button_->setEnabled(false);
  connect(cancel_button_, &QPushButton::clicked, this, &NavigateToPosePanel::onCancel);
  button_layout->addWidget(cancel_button_);
  
  main_layout->addLayout(button_layout);
  
  setLayout(main_layout);
}

void NavigateToPosePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  
  QString action_name;
  if (config.mapGetString("action_name", &action_name) && action_name_edit_) {
    action_name_edit_->setText(action_name);
    current_action_name_ = action_name.toStdString();
  }
  
  QString frame_id;
  if (config.mapGetString("frame_id", &frame_id) && frame_id_edit_) {
    frame_id_edit_->setText(frame_id);
  }
}

void NavigateToPosePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  
  if (action_name_edit_) {
    config.mapSetValue("action_name", action_name_edit_->text());
  }
  if (frame_id_edit_) {
    config.mapSetValue("frame_id", frame_id_edit_->text());
  }
}

void NavigateToPosePanel::onSendGoal()
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
  
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  
  goal_msg.pose = getCurrentPose();
  
  goal_msg.behavior_tree = behavior_tree_edit_->text().toStdString();
  
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  
  send_goal_options.goal_response_callback = 
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
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
    std::bind(&NavigateToPosePanel::onActionResult, this, std::placeholders::_1);
  
  action_client_->async_send_goal(goal_msg, send_goal_options);
  
  if (send_goal_button_) {
    send_goal_button_->setEnabled(false);
  }
}

void NavigateToPosePanel::onCancel()
{
  if (goal_handle_ && goal_active_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
}

void NavigateToPosePanel::onActionNameChanged()
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
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
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

geometry_msgs::msg::PoseStamped NavigateToPosePanel::getCurrentPose()
{
  geometry_msgs::msg::PoseStamped pose;
  
  if (!node_ptr_ || !frame_id_edit_ || !pos_x_spin_ || !pos_y_spin_ || !pos_z_spin_ || !yaw_spin_) {
    return pose;
  }
  
  pose.header.stamp = node_ptr_->get_raw_node()->now();
  pose.header.frame_id = frame_id_edit_->text().toStdString();
  
  pose.pose.position.x = pos_x_spin_->value();
  pose.pose.position.y = pos_y_spin_->value();
  pose.pose.position.z = pos_z_spin_->value();
  
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_spin_->value());
  pose.pose.orientation.x = q.getX();
  pose.pose.orientation.y = q.getY();
  pose.pose.orientation.z = q.getZ();
  pose.pose.orientation.w = q.getW();
  
  return pose;
}

void NavigateToPosePanel::onActionResult(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & /* result */)
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
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::NavigateToPosePanel, rviz_common::Panel)