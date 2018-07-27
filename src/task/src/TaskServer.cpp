// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include <mutex>
#include <chrono>
#include "task/TaskServer.hpp"

using namespace std::chrono_literals;

TaskServer::TaskServer(const std::string & name)
: Node(name), workerThread_(nullptr)
{
  RCLCPP_INFO(get_logger(), "TaskServer::TaskServer");

  commandSub_ = create_subscription<std_msgs::msg::String>(name + "_command",
      std::bind(&TaskServer::onCommandReceived, this, std::placeholders::_1));

  cancelSub_ = create_subscription<std_msgs::msg::String>(name + "_cancel",
      std::bind(&TaskServer::onCancelReceived, this, std::placeholders::_1));

  resultPub_ = this->create_publisher<ResultMsg>(name + "_result");

  startWorkerThread();
}

TaskServer::~TaskServer()
{
  RCLCPP_INFO(get_logger(), "TaskServer::~TaskServer");
  stopWorkerThread();
}

bool
TaskServer::cancelRequested()
{
  return shouldCancel_;
}

void
TaskServer::setCanceled()
{
  shouldCancel_ = false;
}

void
TaskServer::sendResult(const ResultMsg & result)
{
  resultPub_->publish(result);
}

void
TaskServer::workerThread()
{
  RCLCPP_INFO(get_logger(), "TaskServer::workerThread: enter");

  std::mutex m;
  std::unique_lock<std::mutex> lock(m);

  do {
    cv_.wait_for(lock, 10ms);

    if (shouldExecute_) {
      RCLCPP_INFO(get_logger(), "TaskServer::workerThread: shouldExecute");
      auto command = std::make_shared<std_msgs::msg::String>();
      command->data = "Command to execute";
      Status status = execute(command);
      shouldExecute_ = false;
    }
  } while (rclcpp::ok());

  RCLCPP_INFO(get_logger(), "TaskServer::workerThread: exit");
}

void
TaskServer::startWorkerThread()
{
  RCLCPP_INFO(get_logger(), "TaskServer::startWorkerThread");
  workerThread_ = new std::thread(&TaskServer::workerThread, this);
}

void
TaskServer::stopWorkerThread()
{
  RCLCPP_INFO(get_logger(), "TaskServer::stopWorkerThread");
  workerThread_->join();
  delete workerThread_;
  workerThread_ = nullptr;
}

void
TaskServer::onCommandReceived(const CommandMsg::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "TaskServer::onCommandReceived: \"%s\"", msg->data.c_str())
  shouldExecute_ = true;
  cv_.notify_one();
}

void
TaskServer::onCancelReceived(const CancelMsg::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "TaskServer::onCancelReceived: \"%s\"", msg->data.c_str())
  shouldCancel_ = true;
  cv_.notify_one();
}
