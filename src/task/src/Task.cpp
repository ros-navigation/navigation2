// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/Task.hpp"

Task::Task(const std::string & name)
: Node(name), workerThread_(nullptr), stopWorkerThread_(false)
{
  RCLCPP_INFO(get_logger(), "Task::Task");

  cmdSub_ = create_subscription<std_msgs::msg::String>("TaskCmd",
    std::bind(&Task::onCmdReceived, this, std::placeholders::_1));
}

Task::~Task()
{
  RCLCPP_INFO(get_logger(), "Task::~Task");
  if (workerThread_ != nullptr)
    cancel();
}

void
Task::execute()
{
  RCLCPP_INFO(get_logger(), "Task::execute");
  stopWorkerThread_ = false;
  workerThread_ = new std::thread(&Task::workerThread, this);
}

void
Task::cancel()
{
  RCLCPP_INFO(get_logger(), "Task::cancel");
  stopWorkerThread_ = true;
  workerThread_->join();

  delete workerThread_;
  workerThread_ = nullptr;
}

void
Task::onCmdReceived(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Task::onCmdReceived: \"%s\"", msg->data.c_str())

  if (msg->data.compare("ExecuteTask") == 0) {
    execute();
  } else if (msg->data.compare("CancelTask") == 0) {
    cancel();
  } else {
    RCLCPP_INFO(get_logger(), "Task::onCmdReceived: invalid command: \"%s\"",
      msg->data.c_str())
  }
}
