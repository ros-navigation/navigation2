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

#ifndef NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QAbstractTransition>
#include <QDialog>
#include <QBasicTimer>

#include <memory>
#include <string>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class QPushButton;

namespace nav2_rviz_plugins
{

/// Custom Event to track state of ROS Action
struct ROSActionEvent : public QEvent
{
  explicit ROSActionEvent(const bool & is_active)
  : QEvent(QEvent::Type(QEvent::User + 1)),
    is_active_(is_active) {}

  bool is_active_;
};

/// Custom Transition to check whether ROS Action state has changed
class ROSActionTransition : public QAbstractTransition
{
public:
  explicit ROSActionTransition(bool initial_status)
  : status_(initial_status)
  {}

  ~ROSActionTransition() {}

protected:
  virtual bool eventTest(QEvent * e)
  {
    if (e->type() != QEvent::Type(QEvent::User + 1)) {  // ROSActionEvent
      return false;
    }
    ROSActionEvent * action_event = static_cast<ROSActionEvent *>(e);
    return status_ != action_event->is_active_;
  }

  virtual void onTransition(QEvent *) {}
  bool status_;
};

/// Panel to interface to the nav2 stack
class Nav2Panel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Nav2Panel(QWidget * parent = 0);
  virtual ~Nav2Panel();

  void onInitialize() override;

  /// Load and save configuration data
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onStartup();
  void onShutdown();
  void onCancel();

private:
  void loadLogFiles();
  void onCancelButtonPressed();
  void timerActionEvent();

  void startNavigation(geometry_msgs::msg::PoseStamped::SharedPtr pose);
  using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  // The (non-spinning) client node used to invoke the action client
  rclcpp::Node::SharedPtr client_node_;

  // Node to spin for goal pose and timer
  rclcpp::Node::SharedPtr goal_node_;

  // Executor needed to spin node in thread
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> thread_;

  // Subscriber for goal pose updates to action client
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // Timer to check goal status and send ROSActionEvent
  rclcpp::TimerBase::SharedPtr timer_;

  // The NavigateToPose action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

  // Goal-related state
  nav2_msgs::action::NavigateToPose::Goal goal_;
  GoalHandle::SharedPtr goal_handle_;

  // A timer used to check on the completion status of the action

  // The client used to control the nav2 stack
  nav2_lifecycle_manager::LifecycleManagerClient client_;

  QPushButton * start_stop_button_{nullptr};

  QStateMachine machine_;

  QState * initial_{nullptr};
  QState * starting_{nullptr};
  QState * stopping_{nullptr};
  // The following states are added to allow for the state of the button to only expose shutdown
  // while the NavigateToPose action is not active. While running, the user will be allowed to
  // cancel the action. The ROSActionTransition allows for the state of the action to be detected
  // and the button state to change automatically. The additional states is_canceled and
  // is_succesful support the transitions from running into states that can transition to shutdown
  // but do nothing upon entrance.
  QState * running_{nullptr};
  QState * canceled_{nullptr};
  QState * completed_{nullptr};
};

}  // namespace nav2_rviz_plugins

#endif  //  NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
