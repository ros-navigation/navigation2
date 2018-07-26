// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/TaskServer.hpp"

TaskServer::TaskServer(const std::string & name)
: Node(name), workerThread_(nullptr), stopWorkerThread_(false), running_(false)
{
  RCLCPP_INFO(get_logger(), "TaskServer::TaskServer");

  goalSub_ = create_subscription<std_msgs::msg::String>(name + "_goal",
    std::bind(&TaskServer::onGoalReceived, this, std::placeholders::_1));

  cancelSub_ = create_subscription<std_msgs::msg::String>(name + "_cancel",
    std::bind(&TaskServer::onCancelReceived, this, std::placeholders::_1));

  resultPub_ = this->create_publisher<Result>(name + "_result");
  feedbackPub_ = this->create_publisher<Feedback>(name + "_status");
  statusPub_ = this->create_publisher<Status>(name + "_feedback");
}

TaskServer::~TaskServer()
{
  RCLCPP_INFO(get_logger(), "TaskServer::~TaskServer");
  if (workerThread_ != nullptr)
    stop();
}

void
TaskServer::start()
{
  RCLCPP_INFO(get_logger(), "TaskServer::start");

  if (running_) {
    RCLCPP_INFO(get_logger(), "TaskServer::start: thread already running");
  } else {
    stopWorkerThread_ = false;
    workerThread_ = new std::thread(&TaskServer::workerThread, this);
	running_ = true;
  }
}

void
TaskServer::stop()
{
  RCLCPP_INFO(get_logger(), "TaskServer::stop");

  if (!running_) {
    RCLCPP_INFO(get_logger(), "TaskServer::stop: thread already stopped");
  } else {
    stopWorkerThread_ = true;
    workerThread_->join();
    delete workerThread_;
    workerThread_ = nullptr;
	running_ = false;
  }
}

void
TaskServer::onGoalReceived(const Goal::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "TaskServer::onGoalReceived: \"%s\"", msg->data.c_str())

  // TODO: save the msg, start the worker thread, which passes the msg to the user's callback
  start();
}

void
TaskServer::onCancelReceived(const GoalID::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "TaskServer::onCancelReceived: \"%s\"", msg->data.c_str())
  stop();
}
