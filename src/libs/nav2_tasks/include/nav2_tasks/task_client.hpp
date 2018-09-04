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

#ifndef NAV2_TASKS__TASK_CLIENT_HPP_
#define NAV2_TASKS__TASK_CLIENT_HPP_

#include <atomic>
#include <condition_variable>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_tasks/task_status.hpp"

namespace nav2_tasks
{

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
    statusSub_ = node_->create_subscription<StatusMsg>(name + "_status",
        std::bind(&TaskClient::onStatusReceived, this, std::placeholders::_1));
  }

  ~TaskClient()
  {
  }

  // The client can tell the TaskServer to execute its operation
  void sendCommand(const typename CommandMsg::SharedPtr msg)
  {
    resultReceived_ = false;
    statusReceived_ = false;
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
  TaskStatus waitForResult(
    typename ResultMsg::SharedPtr & result,
    std::chrono::milliseconds duration)
  {
    // Wait for a status message to come in
    std::unique_lock<std::mutex> lock(statusMutex_);
    if (!cvStatus_.wait_for(lock, std::chrono::milliseconds(duration),
      [&] {return statusReceived_ == true;}))
    {
      return RUNNING;
    }

    // We've got a status message, indicating that the server task has finished (succeeded,
    // failed, or canceled)
    switch (statusMsg_->result) {
      // If the task has failed or has been canceled, no result message is forthcoming and we
      // can propagate the status code, using the TaskStatus type rather than the message-level
      // implementation type
      case nav2_tasks::msg::TaskStatus::FAILED:
      case nav2_tasks::msg::TaskStatus::CANCELED:
        return static_cast<TaskStatus>(statusMsg_->result);

      case nav2_tasks::msg::TaskStatus::SUCCEEDED:
        {
          // The result message may be here already or it may come *after* the status
          // message. If it's here, the wait will be satisfied immediately. Otherwise
          // we'll wait for a bit longer to see if it comes in.
          std::unique_lock<std::mutex> lock(resultMutex_);
          if (cvResult_.wait_for(lock, std::chrono::milliseconds(100),
            [&] {return resultReceived_ == true;}))
          {
            result = resultMsg_;
            resultReceived_ = false;
            return SUCCEEDED;
          }

          // Give up since we never received the result message
          return FAILED;
        }

      default:
        throw std::logic_error("Invalid status value from TaskServer");
    }

    // Not reachable; added to avoid a warning
    return FAILED;
  }

protected:
  // The result of this task
  typename ResultMsg::SharedPtr resultMsg_;

  // These messages are internal to the TaskClient implementation
  typedef std_msgs::msg::String CancelMsg;
  typedef nav2_tasks::msg::TaskStatus StatusMsg;
  StatusMsg::SharedPtr statusMsg_;

  // Variables to handle the communication of the status message to the waitForResult thread
  std::mutex statusMutex_;
  std::atomic<bool> statusReceived_;
  std::condition_variable cvStatus_;

  // Variables to handle the communication of the result message to the waitForResult thread
  std::mutex resultMutex_;
  std::atomic<bool> resultReceived_;
  std::condition_variable cvResult_;

  // Called when the TaskServer has sent its result
  void onResultReceived(const typename ResultMsg::SharedPtr resultMsg)
  {
    {
      std::lock_guard<std::mutex> lock(resultMutex_);
      resultMsg_ = resultMsg;
      resultReceived_ = true;
    }

    cvResult_.notify_one();
  }

  // Called when the TaskServer sends it status code (success or failure)
  void onStatusReceived(const StatusMsg::SharedPtr statusMsg)
  {
    {
      std::lock_guard<std::mutex> lock(statusMutex_);
      statusMsg_ = statusMsg;
      statusReceived_ = true;
    }

    cvStatus_.notify_one();
  }

  // The TaskClient isn't itself a node, so needs to know which one to use
  rclcpp::Node * node_;

  // The client's publishers: the command and cancel messages
  typename rclcpp::Publisher<CommandMsg>::SharedPtr commandPub_;
  rclcpp::Publisher<CancelMsg>::SharedPtr cancelPub_;

  // The client's subscriptions: result, feedback, and status
  typename rclcpp::Subscription<ResultMsg>::SharedPtr resultSub_;
  rclcpp::Subscription<StatusMsg>::SharedPtr statusSub_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__TASK_CLIENT_HPP_
