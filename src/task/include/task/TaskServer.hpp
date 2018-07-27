// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKSERVER_HPP_
#define TASK__TASKSERVER_HPP_

#include <atomic>
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
  typedef std_msgs::msg::String FeedbackMsg;
  typedef std_msgs::msg::String StatusMsg;

  // TODO(mjeronimo): For now, the user's class overrides this virtual method. Need 
  // to move this to the constructor w/ a callback.
  virtual void execute(/*command message*/) = 0;
  typedef std::function<void (const std_msgs::msg::String &)> ExecuteCallback;

  // The user's execute method can check if the client is requesting a cancel
  bool cancelRequested();
  void setCanceled();

  // TODO(mjeronimo): other methods useful to the user's execute implementation
  // (ala ActionLib). May not need feedback; may only need success/failure/running.
  //
  //   void publishStatus();
  //   void publishResult();
  //   void publishFeedback();

protected:
  // The pointer to our private worker thread
  std::thread * workerThread_;

  // This class has the worker thread body which calls the user's execute() callback
  void workerThread();

  // Convenience routes for starting and stopping the worker thread (used from the ctor and dtor)
  void startWorkerThread();
  void stopWorkerThread();

  // The callbacks for our subscribers
  void onCommandReceived(const CommandMsg::SharedPtr msg);
  void onCancelReceived(const CancelMsg::SharedPtr msg);

  // The subscribers: command and cancel
  rclcpp::Subscription<CommandMsg>::SharedPtr commandSub_;
  rclcpp::Subscription<CancelMsg>::SharedPtr cancelSub_;

  // The publishers: result, feedback, and status
  rclcpp::Publisher<ResultMsg>::SharedPtr resultPub_;
  rclcpp::Publisher<FeedbackMsg>::SharedPtr feedbackPub_;
  rclcpp::Publisher<StatusMsg>::SharedPtr statusPub_;
};

#endif  // TASK__TASKSERVER_HPP_
