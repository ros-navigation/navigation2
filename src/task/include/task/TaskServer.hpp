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

  // Execute
  typedef std::function<void (const std_msgs::msg::String&)> ExecuteCallback;

  typedef std_msgs::msg::String Goal;
  typedef std_msgs::msg::String GoalID;
  typedef std_msgs::msg::String Result;
  typedef std_msgs::msg::String Feedback;
  typedef std_msgs::msg::String Status;

  // void publishStatus();
  // void publishResult();
  // void publishFeedback();

  // Timer, status frequency

protected:
  void start(); 
  void stop();

  virtual void workerThread() = 0;

  std::thread *workerThread_;
  std::atomic<bool> stopWorkerThread_;
  std::atomic<bool> running_;

  void onGoalReceived(const Goal::SharedPtr msg);
  void onCancelReceived(const GoalID::SharedPtr msg);

  rclcpp::Subscription<Goal>::SharedPtr goalSub_;
  rclcpp::Subscription<GoalID>::SharedPtr cancelSub_;

  rclcpp::Publisher<Result>::SharedPtr resultPub_;
  rclcpp::Publisher<Feedback>::SharedPtr feedbackPub_;
  rclcpp::Publisher<Status>::SharedPtr statusPub_;
};

#endif  // TASK__TASKSERVER_HPP_
