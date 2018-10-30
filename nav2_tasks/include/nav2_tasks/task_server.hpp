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

#ifndef NAV2_TASKS__TASK_SERVER_HPP_
#define NAV2_TASKS__TASK_SERVER_HPP_

#include <atomic>
#include <condition_variable>
#include <thread>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_tasks/task_status.hpp"

namespace nav2_tasks
{

template<class CommandMsg, class ResultMsg>
const char * getTaskName();

template<class CommandMsg, class ResultMsg>
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const std::string & name, bool autoStart = true)
  : Node(name),
    workerThread_(nullptr),
    commandReceived_(false),
    eptr_(nullptr)
  {
    std::string taskName = getTaskName<CommandMsg, ResultMsg>();
    commandSub_ = create_subscription<CommandMsg>(taskName + "_command",
        std::bind(&TaskServer::onCommandReceived, this, std::placeholders::_1));

    cancelSub_ = create_subscription<std_msgs::msg::Empty>(taskName + "_cancel",
        std::bind(&TaskServer::onCancelReceived, this, std::placeholders::_1));

    resultPub_ = this->create_publisher<ResultMsg>(taskName + "_result");
    statusPub_ = this->create_publisher<StatusMsg>(taskName + "_status");

    if (autoStart) {
      startWorkerThread();
    }
  }

  virtual ~TaskServer()
  {
    stopWorkerThread();
  }

  virtual TaskStatus execute(const typename CommandMsg::SharedPtr command) = 0;

  // The user's execute method can check if the client is requesting a cancel
  bool cancelRequested()
  {
    return cancelReceived_;
  }

  void setCanceled()
  {
    cancelReceived_ = false;
  }

  void setResult(const ResultMsg & result)
  {
    resultMsg_ = result;
  }

protected:
  typename CommandMsg::SharedPtr commandMsg_;
  ResultMsg resultMsg_;

  // These messages are internal to the TaskClient implementation
  typedef std_msgs::msg::Empty CancelMsg;
  typedef nav2_msgs::msg::TaskStatus StatusMsg;

  // The pointer to our private worker thread
  std::thread * workerThread_;

  // This class has the worker thread body which calls the user's execute() callback
  void workerThread()
  {
    do {
      std::unique_lock<std::mutex> lock(commandMutex_);
      if (cvCommand_.wait_for(lock, std::chrono::milliseconds(100),
        [&] {return commandReceived_ == true;}))
      {
        nav2_msgs::msg::TaskStatus statusMsg;
        TaskStatus status = TaskStatus::FAILED;

        // Call the user's overridden method
        try {
          status = execute(commandMsg_);
        } catch (...) {
          statusMsg.result = nav2_msgs::msg::TaskStatus::FAILED;
          statusPub_->publish(statusMsg);

          // Save the exception so that we can propagate it back to the thread owning
          // this object (the task server)
          eptr_ = std::current_exception();

          // TODO(mjeronimo): using rclcpp:shutdown is the only way I know so far to tell
          // ROS to stop this node from spinning so that it will be destroyed and we can
          // propagate the exception from the node's destructor. I'd rather have a way to
          // shutdown just this node, but at least this is better than having the node
          // spinning even when a node's thread has terminated with a fault/exception
          rclcpp::shutdown();
        }

        // Reset the execution flag now that we've executed the task
        commandReceived_ = false;

        // Check the result of the user's function and send the
        // appropriate message
        if (status == TaskStatus::SUCCEEDED) {
          // If the task succeeded, first publish the result message
          resultPub_->publish(resultMsg_);

          // Then send the success code
          statusMsg.result = nav2_msgs::msg::TaskStatus::SUCCEEDED;
          statusPub_->publish(statusMsg);
        } else if (status == TaskStatus::FAILED) {
          // Otherwise, send the failure code
          statusMsg.result = nav2_msgs::msg::TaskStatus::FAILED;
          statusPub_->publish(statusMsg);
        } else if (status == TaskStatus::CANCELED) {
          // Or the canceled code
          statusMsg.result = nav2_msgs::msg::TaskStatus::CANCELED;
          statusPub_->publish(statusMsg);

          cancelReceived_ = false;
        } else {
          throw std::logic_error("Unexpected status return from task");
        }
      }
    } while (rclcpp::ok());
  }

  // TODO(mjeronimo): Make explicit start and stop calls to control
  // the worker thread

  // Convenience routines for starting and stopping the worker thread.
  void startWorkerThread()
  {
    workerThread_ = new std::thread(&TaskServer::workerThread, this);
  }

  void stopWorkerThread()
  {
    workerThread_->join();
    delete workerThread_;
    workerThread_ = nullptr;

    // If there was an exception during execution, rethrow the exception so
    // that the owning thread (the thread that created this object) receives it
    if (eptr_ != nullptr) {
      std::rethrow_exception(eptr_);
    }
  }

  // Variables to handle the communication of the command to the execute thread
  std::mutex commandMutex_;
  bool commandReceived_;
  std::condition_variable cvCommand_;

  // Variables to handle the communication of the cancel request to the execute thread
  std::atomic<bool> cancelReceived_;
  std::condition_variable cvCancel_;

  // The callbacks for our subscribers
  void onCommandReceived(const typename CommandMsg::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(commandMutex_);
      commandMsg_ = msg;
      commandReceived_ = true;
    }

    cvCommand_.notify_one();
  }

  void onCancelReceived(const CancelMsg::SharedPtr /*msg*/)
  {
    cancelReceived_ = true;
    cvCancel_.notify_one();
  }

  // The subscribers: command and cancel
  typename rclcpp::Subscription<CommandMsg>::SharedPtr commandSub_;
  rclcpp::Subscription<CancelMsg>::SharedPtr cancelSub_;

  // The publishers for the result from this task
  typename rclcpp::Publisher<ResultMsg>::SharedPtr resultPub_;
  typename rclcpp::Publisher<StatusMsg>::SharedPtr statusPub_;

  std::exception_ptr eptr_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__TASK_SERVER_HPP_
