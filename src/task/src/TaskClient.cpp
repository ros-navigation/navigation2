// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include <thread>
#include <chrono>
#include "task/TaskClient.hpp"

TaskClient::TaskClient(const std::string & name, rclcpp::Node * node)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "TaskClient::TaskClient: %s", name.c_str());

  commandPub_ = node_->create_publisher<CommandMsg>(name + "_command");
  cancelPub_ = node_->create_publisher<CancelMsg>(name + "_cancel");

  resultSub_ = node_->create_subscription<std_msgs::msg::String>(name + "_result",
      std::bind(&TaskClient::onResultReceived, this, std::placeholders::_1));

  feedbackSub_ = node_->create_subscription<std_msgs::msg::String>(name + "_feedback",
      std::bind(&TaskClient::onFeedbackReceived, this, std::placeholders::_1));

  statusSub_ = node_->create_subscription<std_msgs::msg::String>(name + "_status",
      std::bind(&TaskClient::onStatusReceived, this, std::placeholders::_1));
}

TaskClient::~TaskClient()
{
  RCLCPP_INFO(node_->get_logger(), "TaskClient::~TaskClient");
}

void
TaskClient::execute()
{
  std_msgs::msg::String msg;
  msg.data = "Hello, World!";

  RCLCPP_INFO(node_->get_logger(), "TaskClient::execute: commandPub_: %p", commandPub_);
  commandPub_->publish(msg);
}

void
TaskClient::cancel()
{
  std_msgs::msg::String msg;
  msg.data = "Goodbye, World!";

  cancelPub_->publish(msg);
}

TaskClient::Status
TaskClient::waitForResult(const ResultMsg::SharedPtr & result)
{
  RCLCPP_INFO(node_->get_logger(), "TaskClient::waitForResult");

  static int i;

  // Simulate the task running for a bit
  if (i < 5) {
    i++;
    RCLCPP_INFO(node_->get_logger(), "TaskClient::waitForResult: not done");
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    return RUNNING; 
  }

  i = 0;
  RCLCPP_INFO(node_->get_logger(), "TaskClient::waitForResult: done");
  result->data = "Some fake result from TaskClient";
  return SUCCEEDED;
}

void
TaskClient::onResultReceived(const ResultMsg::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "TaskClient::onResultReceived: %s", msg->data.c_str());
}

void
TaskClient::onFeedbackReceived(const FeedbackMsg::SharedPtr msg)
{
}

void
TaskClient::onStatusReceived(const StatusMsg::SharedPtr msg)
{
}
