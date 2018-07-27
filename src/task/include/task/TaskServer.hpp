// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKSERVER_HPP_
#define TASK__TASKSERVER_HPP_

#include <atomic>
#include <condition_variable>
#include <thread>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const std::string & name);
  virtual ~TaskServer();

  typedef std_msgs::msg::String CommandMsg;
  typedef std_msgs::msg::String CancelMsg;
  typedef std_msgs::msg::String ResultMsg;

  typedef enum { SUCCEEDED, FAILED, CANCELED } Status;

  virtual Status execute(const CommandMsg::SharedPtr command) = 0;

  // The user's execute method can check if the client is requesting a cancel
  bool cancelRequested();
  void setCanceled();
  void sendResult(const ResultMsg & result);

protected:
  // The pointer to our private worker thread
  std::thread * workerThread_;

  // This class has the worker thread body which calls the user's execute() callback
  void workerThread();

  // Convenience routes for starting and stopping the worker thread (used from the ctor and dtor)
  void startWorkerThread();
  void stopWorkerThread();

  std::condition_variable cv_;
  std::atomic<bool> shouldCancel_;
  std::atomic<bool> shouldExecute_;

  // The callbacks for our subscribers
  void onCommandReceived(const CommandMsg::SharedPtr msg);
  void onCancelReceived(const CancelMsg::SharedPtr msg);

  // The subscribers: command and cancel
  rclcpp::Subscription<CommandMsg>::SharedPtr commandSub_;
  rclcpp::Subscription<CancelMsg>::SharedPtr cancelSub_;

  // The publishers for the result from this task
  rclcpp::Publisher<ResultMsg>::SharedPtr resultPub_;
};

#endif  // TASK__TASKSERVER_HPP_
