// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TASK__TASKCLIENT_HPP_
#define TASK__TASKCLIENT_HPP_

#include <atomic>
#include <condition_variable>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "task/TaskStatus.hpp"

template<class CommandMsg, class ResultMsg>
class TaskClient
{
public:
  TaskClient(const std::string & name, rclcpp::Node * node)
  : node_(node)
  {
    // Create the publishers
    commandPub_ = node_->create_publisher<CommandMsg>(name + "_command");
    cancelPub_ = node_->create_publisher<CancelMsg>(name + "_cancel");

    // Create the subscribers
    resultSub_ = node_->create_subscription<ResultMsg>(name + "_result",
        std::bind(&TaskClient::onResultReceived, this, std::placeholders::_1));
    statusSub_ = node_->create_subscription<std_msgs::msg::String>(name + "_status",
        std::bind(&TaskClient::onStatusReceived, this, std::placeholders::_1));
  }

  ~TaskClient()
  {
  }

  // The client can tell the TaskServer to execute its operation
  void executeAsync(const typename CommandMsg::SharedPtr msg)
  {
    taskSucceeded_ = false;
    receivedNewMsg_ = false;
    commandPub_->publish(msg);
  }

  // An in-flight operation can be canceled
  void cancel()
  {
    std_msgs::msg::String msg;
    msg.data = "Goodbye, World!";
    cancelPub_->publish(msg);
  }

  // The client can wait for a result with a timeout
  TaskStatus waitForResult(typename ResultMsg::SharedPtr & result, unsigned int milliseconds)
  {
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    std::cv_status timeoutStatus =
      cv_.wait_for(lock, std::chrono::milliseconds(milliseconds));

    if (timeoutStatus == std::cv_status::timeout) {
      return RUNNING;
    }

    // TODO(orduno): possible race condition between receiving status message and actual data
    //               for now implemented a method using a flag, but might want to review further
    if (taskSucceeded_ && receivedNewMsg_) {
      result = result_;
      receivedNewMsg_ = false;
      return SUCCEEDED;
    }

    return FAILED;
  }

protected:
  // These messages are internal to the TaskClient implementation
  typedef std_msgs::msg::String CancelMsg;
  typedef std_msgs::msg::String StatusMsg;

  std::condition_variable cv_;
  typename ResultMsg::SharedPtr result_;

  // Called when the TaskServer has sent its result
  void onResultReceived(const typename ResultMsg::SharedPtr msg)
  {
    // Save it off
    result_ = msg;
    receivedNewMsg_ = true;
  }

  // Called when the TaskServer sends it status code (success or failure)
  void onStatusReceived(const StatusMsg::SharedPtr /*msg*/)
  {
    taskSucceeded_ = true;  // msg.data.equals("Success");
    cv_.notify_one();
  }

  // The TaskClient isn't itself a node, so needs to know which one to use
  rclcpp::Node * node_;

  // The client's publishers: the command and cancel messages
  typename rclcpp::Publisher<CommandMsg>::SharedPtr commandPub_;
  rclcpp::Publisher<CancelMsg>::SharedPtr cancelPub_;

  // The client's subscriptions: result, feedback, and status
  typename rclcpp::Subscription<ResultMsg>::SharedPtr resultSub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr statusSub_;

  // A convenience variable for whether the task succeeded or not
  bool taskSucceeded_;

  std::atomic<bool> receivedNewMsg_;
};

#endif  // TASK__TASKCLIENT_HPP_
