// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef NAVIGATION__NAVIGATION_HPP_
#define NAVIGATION__NAVIGATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "task/TaskExecutionStateMachineBehavior.hpp"
#include "task/TaskExecutionStateMachine.hpp"

class Navigation : public rclcpp::Node, public TaskExecutionStateMachineBehavior
{
public:
  Navigation();
  virtual ~Navigation();

  /**
   * @brief Execute a navigation task
   */
  void executeNavigationTask(/*const Task * task*/);

  /**
   * @brief Cancel the current, in-flight navigation task, which must have
   * been previously starting using executeNavigationTask.
   */
  void cancelNavigationTask();

  /** @name State Machine Behavior
   * Implementation of the TaskExecutionStateMachineBehavior interface
   */
  ///@{
  void doReadyState();
  void doExecutingState();
  void doCancelingState();
  void doRecoveringState();
  void doAbortingState();
  ///@}

private:
  void onCmdReceived(const std_msgs::msg::String::SharedPtr msg);

private:
  TaskExecutionStateMachine stateMachine_;
  // const NavigationTask * navigationTask_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdSub_;
};

#endif  // NAVIGATION__NAVIGATION_HPP_
