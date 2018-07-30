// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKCLIENT_HPP_
#define TASK__TASKCLIENT_HPP_

#include <atomic>
#include <condition_variable>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

template <class CommandMsg, class ResultMsg>
class TaskClient
{
public:
  TaskClient(const std::string & name, rclcpp::Node * node)
  : node_(node)
  {
    commandPub_ = node_->create_publisher<CommandMsg>(name + "_command");
    cancelPub_ = node_->create_publisher<CancelMsg>(name + "_cancel");

    resultSub_ = node_->create_subscription<std_msgs::msg::String>(name + "_result",
        std::bind(&TaskClient::onResultReceived, this, std::placeholders::_1));

    feedbackSub_ = node_->create_subscription<std_msgs::msg::String>(name + "_feedback",
        std::bind(&TaskClient::onFeedbackReceived, this, std::placeholders::_1));

    statusSub_ = node_->create_subscription<std_msgs::msg::String>(name + "_status",
        std::bind(&TaskClient::onStatusReceived, this, std::placeholders::_1));
  }

  ~TaskClient()
  {
  }

  typedef std_msgs::msg::String CancelMsg;
  typedef std_msgs::msg::String FeedbackMsg;
  typedef std_msgs::msg::String StatusMsg;

  // The client can tell the TaskServer to execute it's operation
  void execute()
  {
    std_msgs::msg::String msg;
    msg.data = "Hello, World!";
    commandPub_->publish(msg);
  }

  // An in-flight operation can be cancelled
  void cancel()
  {
    std_msgs::msg::String msg;
    msg.data = "Goodbye, World!";
    cancelPub_->publish(msg);
  }

  typedef enum { SUCCEEDED, FAILED, RUNNING } Status;
  Status waitForResult(const typename ResultMsg::SharedPtr &result)
  { 
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    //cv_.wait_for(lock /*timeout*/);
    cv_.wait(lock);

    // TODO(mjeronimo) fix the copies
    *result = result_;
    return SUCCEEDED;
  }

protected:
  std::condition_variable cv_;
  ResultMsg result_;

  // The callbacks for the subscriptions
  void onResultReceived(const typename ResultMsg::SharedPtr msg)
  {
    // Save the result and signal the client's waitForResult thread
    result_ = *msg;
    cv_.notify_one();
  }

  void onFeedbackReceived(const FeedbackMsg::SharedPtr /*msg*/)
  {
  }

  void onStatusReceived(const StatusMsg::SharedPtr /*msg*/)
  {
  }

  // The TaskClient isn't itself a node, so needs to know which one to use
  rclcpp::Node * node_;

  // The client's publishers: the command and cancel messages
  typename rclcpp::Publisher<CommandMsg>::SharedPtr commandPub_;
  rclcpp::Publisher<CancelMsg>::SharedPtr cancelPub_;

  // The client's subscriptions: result, feedback, and status
  typename rclcpp::Subscription<ResultMsg>::SharedPtr resultSub_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedbackSub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr statusSub_;
};

#endif  // TASK__TASKCLIENT_HPP_
