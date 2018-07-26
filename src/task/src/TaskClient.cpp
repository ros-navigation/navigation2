// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/TaskClient.hpp"

TaskClient::TaskClient(const std::string & name, rclcpp::Node * node)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "TaskClient::TaskClient: %s", name.c_str());

  goalPub_ = node_->create_publisher<Goal>(name + "_goal");
  cancelPub_ = node_->create_publisher<GoalID>(name + "_cancel");

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

  RCLCPP_INFO(node_->get_logger(), "TaskClient::execute: goalPub_: %p", goalPub_);
  goalPub_->publish(msg);
}

void 
TaskClient::cancel()
{
  std_msgs::msg::String msg;
  msg.data = "Goodbye, World!";

  cancelPub_->publish(msg);
}

void 
TaskClient::onResultReceived(const Result::SharedPtr msg)
{
}

void 
TaskClient::onFeedbackReceived(const Feedback::SharedPtr msg)
{
}

void 
TaskClient::onStatusReceived(const Status::SharedPtr msg)
{
}
