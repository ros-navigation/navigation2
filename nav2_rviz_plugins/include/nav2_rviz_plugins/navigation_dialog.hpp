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

#ifndef NAV2_RVIZ_PLUGINS__NAVIGATION_DIALOG_HPP_
#define NAV2_RVIZ_PLUGINS__NAVIGATION_DIALOG_HPP_

#include <QDialog>
#include <QBasicTimer>

#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class QPushButton;

class NavigationDialog : public QDialog
{
  Q_OBJECT

public:
  explicit NavigationDialog(QWidget * parent = 0);

  void startNavigation(double x, double y, double theta, std::string & frame);

protected:
  void timerEvent(QTimerEvent * event);
  geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);

private slots:
  void onCancelButtonPressed();

private:
  using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;

  // The NavigateToPose action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

  // Goal-related state
  nav2_msgs::action::NavigateToPose::Goal goal_;
  GoalHandle::SharedPtr goal_handle_;

  // A timer used to check on the completion status of the action
  QBasicTimer timer_;
};

#endif  //  NAV2_RVIZ_PLUGINS__NAVIGATION_DIALOG_HPP_
