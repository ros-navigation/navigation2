// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKSERVER_HPP_
#define TASK__TASKSERVER_HPP_

#include <atomic>
#include <condition_variable>
#include <thread>
#include <string>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

typedef enum { TASK_SUCCEEDED, TASK_FAILED, TASK_CANCELED } TaskStatus;

template <class CommandMsg, class ResultMsg>
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const std::string & name)
  : Node(name), workerThread_(nullptr)
  {
    commandSub_ = create_subscription<std_msgs::msg::String>(name + "_command",
        std::bind(&TaskServer::onCommandReceived, this, std::placeholders::_1));

    cancelSub_ = create_subscription<std_msgs::msg::String>(name + "_cancel",
        std::bind(&TaskServer::onCancelReceived, this, std::placeholders::_1));

    resultPub_ = this->create_publisher<ResultMsg>(name + "_result");

    startWorkerThread();
  }

  virtual ~TaskServer()
  {
    stopWorkerThread();
  }

  typedef std_msgs::msg::String CancelMsg;
  typedef enum { SUCCEEDED, FAILED, CANCELED } Status;

  virtual Status execute(const typename CommandMsg::SharedPtr command) = 0;

  // The user's execute method can check if the client is requesting a cancel
  bool cancelRequested()
  {
    return shouldCancel_;
  }

  void setCanceled()
  {
    shouldCancel_ = false;
  }

  void setResult(const ResultMsg & result)
  {
    resultPub_->publish(result);
  }

protected:
  // The pointer to our private worker thread
  std::thread * workerThread_;

  // This class has the worker thread body which calls the user's execute() callback
  void workerThread()
  {
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    do {
      cv_.wait_for(lock, std::chrono::milliseconds(10));

      if (shouldExecute_) {
        auto command = std::make_shared<std_msgs::msg::String>();
        command->data = "Command to execute";
        /*Status status = */ execute(command);
        shouldExecute_ = false;
      }
    } while (rclcpp::ok());
  }

  // Convenience routes for starting and stopping the worker thread (used from the ctor and dtor)
  void startWorkerThread()
  {
    //RCLCPP_INFO(get_logger(), "TaskServer::startWorkerThread");
    workerThread_ = new std::thread(&TaskServer::workerThread, this);
  }

  void stopWorkerThread()
  {
    workerThread_->join();
    delete workerThread_;
    workerThread_ = nullptr;
  }

  std::condition_variable cv_;
  std::atomic<bool> shouldCancel_;
  std::atomic<bool> shouldExecute_;

  // The callbacks for our subscribers
  void onCommandReceived(const typename CommandMsg::SharedPtr /*msg*/)
  {
    shouldExecute_ = true;
    cv_.notify_one();
  }

  void onCancelReceived(const CancelMsg::SharedPtr /*msg*/)
  {
    shouldCancel_ = true;
    cv_.notify_one();
  }

  // The subscribers: command and cancel
  typename rclcpp::Subscription<CommandMsg>::SharedPtr commandSub_;
  rclcpp::Subscription<CancelMsg>::SharedPtr cancelSub_;

  // The publishers for the result from this task
  typename rclcpp::Publisher<ResultMsg>::SharedPtr resultPub_;
};

#endif  // TASK__TASKSERVER_HPP_
