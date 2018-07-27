// License: Apache 2.0. See LICENSE file in root directory.
// Copyright 2018 Intel Corporation. All Rights Reserved.

#ifndef TASK__TASKCLIENT_HPP_
#define TASK__TASKCLIENT_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TaskClient
{
public:
  TaskClient(const std::string & name, rclcpp::Node * node);
  virtual ~TaskClient();

  typedef std_msgs::msg::String CommandMsg;
  typedef std_msgs::msg::String CancelMsg;
  typedef std_msgs::msg::String ResultMsg;
  typedef std_msgs::msg::String FeedbackMsg;
  typedef std_msgs::msg::String StatusMsg;

  // The client can tell the TaskServer to execute it's operation
  void execute();

  // An in-flight operation can be cancelled
  void cancel();

  typedef enum { SUCCEEDED, FAILED, RUNNING } Status;
  Status waitForResult(const ResultMsg::SharedPtr &result);

protected:
  // The callbacks for the subscriptions
  void onResultReceived(const ResultMsg::SharedPtr msg);
  void onFeedbackReceived(const FeedbackMsg::SharedPtr msg);
  void onStatusReceived(const StatusMsg::SharedPtr msg);

  // The TaskClient isn't itself a node, so needs to know which one to use
  rclcpp::Node * node_;

  // The client's publishers: the command and cancel messages
  rclcpp::Publisher<CommandMsg>::SharedPtr commandPub_;
  rclcpp::Publisher<CancelMsg>::SharedPtr cancelPub_;

  // The client's subscriptions: result, feedback, and status
  rclcpp::Subscription<ResultMsg>::SharedPtr resultSub_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedbackSub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr statusSub_;
};

#endif  // TASK__TASKCLIENT_HPP_
