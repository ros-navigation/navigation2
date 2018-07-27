// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#include "task/TaskServer.hpp"

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

using namespace std::chrono_literals;

static std::condition_variable cv;

static std::atomic<bool> shouldCancel;
static std::atomic<bool> shouldExecute;

TaskServer::TaskServer(const std::string & name)
: Node(name), workerThread_(nullptr)
{
  RCLCPP_INFO(get_logger(), "TaskServer::TaskServer");

  commandSub_ = create_subscription<std_msgs::msg::String>(name + "_command",
      std::bind(&TaskServer::onCommandReceived, this, std::placeholders::_1));

  cancelSub_ = create_subscription<std_msgs::msg::String>(name + "_cancel",
      std::bind(&TaskServer::onCancelReceived, this, std::placeholders::_1));

  resultPub_ = this->create_publisher<ResultMsg>(name + "_result");
  feedbackPub_ = this->create_publisher<FeedbackMsg>(name + "_status");
  statusPub_ = this->create_publisher<StatusMsg>(name + "_feedback");

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
  return shouldCancel;
}

void
TaskServer::setCanceled()
{
  shouldCancel = false;
}

void
TaskServer::workerThread()
{
  RCLCPP_INFO(get_logger(), "TaskServer::workerThread: enter");

  std::mutex m;
  std::unique_lock<std::mutex> lock(m);

  do {
    cv.wait_for(lock, 10ms);

    if (shouldExecute) {
      RCLCPP_INFO(get_logger(), "TaskServer::workerThread: shouldExecute");
      execute();
      shouldExecute = false;
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
  shouldExecute = true;
  cv.notify_one();
}

void
TaskServer::onCancelReceived(const CancelMsg::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "TaskServer::onCancelReceived: \"%s\"", msg->data.c_str())
  shouldCancel = true;
  cv.notify_one();
}
