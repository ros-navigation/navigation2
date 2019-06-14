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

#include <QtWidgets>

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_rviz_plugins/navigation_dialog.hpp"

using namespace std::chrono_literals;

void
NavigationDialog::onCancelButtonPressed()
{
  auto future_cancel = action_client_->async_cancel_goal(goal_handle_);

  if (rclcpp::spin_until_future_complete(client_node_, future_cancel) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel goal");
    return;
  }

  timer_.stop();
  hide();
}

NavigationDialog::NavigationDialog(QWidget * parent)
: QDialog(parent)
{
  auto options = rclcpp::NodeOptions().arguments(
    {"__node:=navigation_dialog_action_client"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);
  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(client_node_,
      "NavigateToPose");
  goal_ = nav2_msgs::action::NavigateToPose::Goal();

  QPushButton * cancelButton = new QPushButton(tr("&Cancel"));
  cancelButton->setDefault(true);

  QHBoxLayout * layout = new QHBoxLayout;
  layout->addWidget(cancelButton);

  setMinimumWidth(120);
  setLayout(layout);
  setWindowTitle(tr("Navigating..."));
  setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

  connect(cancelButton, SIGNAL(clicked()), this, SLOT(onCancelButtonPressed()));
}

void
NavigationDialog::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_.timerId()) {
    auto future_result = goal_handle_->async_result();

    if (rclcpp::spin_until_future_complete(client_node_, future_result, 1ms) ==
      rclcpp::executor::FutureReturnCode::TIMEOUT)
    {
      return;
    }

    auto wrapped_result = future_result.get();
    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        timer_.stop();
        hide();
        return;

      case rclcpp_action::ResultCode::ABORTED:
      case rclcpp_action::ResultCode::CANCELED:
      default:
        timer_.stop();
        hide();
        return;
    }
  } else {
    QWidget::timerEvent(event);
  }
}

geometry_msgs::msg::Quaternion
NavigationDialog::orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

void
NavigationDialog::startNavigation(double x, double y, double theta, std::string & frame)
{
  auto pose = geometry_msgs::msg::PoseStamped();

  pose.header.stamp = rclcpp::Clock().now();
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = orientationAroundZAxis(theta);

  action_client_->wait_for_action_server();

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

  timer_.start(100, this);
}
