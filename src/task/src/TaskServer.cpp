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
: Node(name), workerThread_(nullptr), running_(false)
{
  RCLCPP_INFO(get_logger(), "TaskServer::TaskServer");

  goalSub_ = create_subscription<std_msgs::msg::String>(name + "_goal",
    std::bind(&TaskServer::onGoalReceived, this, std::placeholders::_1));

  cancelSub_ = create_subscription<std_msgs::msg::String>(name + "_cancel",
    std::bind(&TaskServer::onCancelReceived, this, std::placeholders::_1));

  resultPub_ = this->create_publisher<Result>(name + "_result");
  feedbackPub_ = this->create_publisher<Feedback>(name + "_status");
  statusPub_ = this->create_publisher<Status>(name + "_feedback");

  start();
}

TaskServer::~TaskServer()
{
  RCLCPP_INFO(get_logger(), "TaskServer::~TaskServer");
  stop();
}

bool
TaskServer::isPreemptRequested()
{
  return shouldCancel;
}

void
TaskServer::setPreempted()
{
  shouldCancel = false;
}

void
TaskServer::workerThread()
{
  RCLCPP_INFO(get_logger(), "TaskServer::workerThread: enter");

  std::mutex m;
  std::unique_lock<std::mutex> lock(m);

  do
  {
    cv.wait_for(lock, 10ms);

	if (shouldExecute)
    {
      RCLCPP_INFO(get_logger(), "TaskServer::workerThread: shouldExecute");
      execute();
      shouldExecute = false;
    }
  } while (rclcpp::ok());

  RCLCPP_INFO(get_logger(), "TaskServer::workerThread: exit");
}

void
TaskServer::start()
{
  RCLCPP_INFO(get_logger(), "TaskServer::start");

  if (running_) {
    RCLCPP_INFO(get_logger(), "TaskServer::start: thread already running");
  } else {
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
  shouldExecute = true;
  cv.notify_one();
}

void
TaskServer::onCancelReceived(const GoalID::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "TaskServer::onCancelReceived: \"%s\"", msg->data.c_str())
  shouldCancel = true;
  cv.notify_one();
}
