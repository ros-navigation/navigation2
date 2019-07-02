// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__EXECUTORS_HPP_
#define NAV2_UTIL__EXECUTORS_HPP_

#include <utility>
#include <memory>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"

namespace nav2_util
{

// Some nav2 nodes perform service requests within service callbacks. This becomes a problem if
// the node is spinning on a single-threaded executor, since waiting for the service response
// blocks further spinning of the node. The request is sent asynchronously, but the response
// comes on an executable waiting on the queue, and nested spinning is currently not supported.

// There is an open issue to address this upstream:
// https://github.com/ros2/rclcpp/issues/773

// The current solution is to use a multi-threaded executor. A second thread will process the
// service response, if service client is assigned to a `Reentrant` callback group.

std::unique_ptr<rclcpp::executor::Executor>
get_executor(const bool service_requests_on_callbacks)
{
  if (service_requests_on_callbacks) {
    // TODO(orduno) It's not required to have two threads processing executables all the time
    //              for a single node. Replace the double-threaded executor with one that
    //              executes callbacks with internal service requests on a separate thread
    //              i.e., an async service executor.

    const unsigned number_of_threads = 2;
    const bool yield_thread_before_execute = false;

    return std::move(
      std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::executor::ExecutorArgs(),
        number_of_threads, yield_thread_before_execute));
  }

  return std::move(std::make_unique<rclcpp::executors::SingleThreadedExecutor>());
}

std::shared_ptr<rclcpp::callback_group::CallbackGroup>
create_nested_request_group(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> nb)
{
  return nb->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__EXECUTORS_HPP_
