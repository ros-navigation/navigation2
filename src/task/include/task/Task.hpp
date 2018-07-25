// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASK_HPP_
#define TASK__TASK_HPP_

#include <atomic>
#include <thread>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Task: public rclcpp::Node
{
public:
  Task(const std::string & name);
  virtual ~Task();

  void execute();
  void cancel();

protected:
  virtual void workerThread() = 0;

  std::thread *workerThread_;
  std::atomic<bool> stopWorkerThread_;

  void onCmdReceived(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdSub_;
};

#endif  // TASK__TASK_HPP_
