// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKSERVER_HPP_
#define TASK__TASKSERVER_HPP_

#include <atomic>
#include <thread>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TaskServer: public rclcpp::Node
{
public:
  TaskServer(const std::string & name);
  virtual ~TaskServer();

  // TODO: The type of this will be received from the template (a String for now)
  typedef std::function<void (const std_msgs::msg::String&)> ExecuteCallback;

  typedef std_msgs::msg::String Goal;
  typedef std_msgs::msg::String GoalID;
  typedef std_msgs::msg::String Result;
  typedef std_msgs::msg::String Feedback;
  typedef std_msgs::msg::String Status;

  // The user's class overrides this virtual method
  virtual void execute/*CB*/(/*goal message*/) = 0;

  // The user's execute method can check if a cancel/preempt is being attempted
  bool isPreemptRequested();
  void setPreempted();

  // TODO: other methods useful to the user's execute implementation (ala ActionLib)
  // void publishStatus();
  // void publishResult();
  // void publishFeedback();

protected:
  // This class has a worker thread that calls the user's execute callback
  void workerThread();

  // The worker thread can be started and stopped (which is done from the ctor and dtor)
  void start(); 
  void stop();

  // The pointer to our private worker thread
  std::thread * workerThread_;
  std::atomic<bool> running_;

  // The callbacks for the subscribers
  void onGoalReceived(const Goal::SharedPtr msg);
  void onCancelReceived(const GoalID::SharedPtr msg);

  // The subscribers: goal and cancel
  rclcpp::Subscription<Goal>::SharedPtr goalSub_;
  rclcpp::Subscription<GoalID>::SharedPtr cancelSub_;

  // The publishers: result, feedback, and status
  rclcpp::Publisher<Result>::SharedPtr resultPub_;
  rclcpp::Publisher<Feedback>::SharedPtr feedbackPub_;
  rclcpp::Publisher<Status>::SharedPtr statusPub_;
};

#endif  // TASK__TASKSERVER_HPP_
