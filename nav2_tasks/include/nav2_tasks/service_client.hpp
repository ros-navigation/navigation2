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
// limitations under the License. Reserved.

#ifndef NAV2_TASKS__SERVICE_CLIENT_HPP_
#define NAV2_TASKS__SERVICE_CLIENT_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace nav2_tasks
{

template<class ServiceT>
class ServiceClient
{
public:
  explicit ServiceClient(const std::string & name)
  {
    node_ = rclcpp::Node::make_shared(name + "_Node");
    client_ = node_->create_client<ServiceT>(name);
  }

  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  typename ResponseType::SharedPtr invoke(
    typename RequestType::SharedPtr & request,
    const std::chrono::seconds timeout = std::chrono::seconds::max())
  {
    auto result_future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future, timeout) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("ServiceClient::async_send_request: service call failed");
    }

    return result_future.get();
  }

  void waitForService(const std::chrono::seconds timeout = std::chrono::seconds::max())
  {
    while (!client_->wait_for_service(timeout)) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("waitForServer: interrupted while waiting for service to appear");
      }
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__SERVICE_CLIENT_HPP_
