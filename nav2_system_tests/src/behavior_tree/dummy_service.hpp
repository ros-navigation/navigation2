// Copyright (c) 2020 Sarthak Mittal
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

#ifndef BEHAVIOR_TREE__DUMMY_SERVICE_HPP_
#define BEHAVIOR_TREE__DUMMY_SERVICE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_system_tests
{

template<class ServiceT>
class DummyService
{
public:
  explicit DummyService(
    const rclcpp::Node::SharedPtr & node,
    std::string service_name)
  : node_(node),
    service_name_(service_name),
    request_count_(0),
    disabled_(false)
  {
    service_ = node->create_service<ServiceT>(
      service_name,
      std::bind(&DummyService::handle_service, this, _1, _2, _3));
  }

  virtual ~DummyService() = default;

  void disable()
  {
    service_.reset();
    disabled_ = true;
  }

  void enable()
  {
    if (disabled_) {
      service_ = node_->create_service<ServiceT>(
        service_name_,
        std::bind(&DummyService::handle_service, this, _1, _2, _3));
      disabled_ = false;
    }
  }

  void reset()
  {
    enable();
    request_count_ = 0;
  }

  int getRequestCount() const
  {
    return request_count_;
  }

protected:
  virtual void fillResponse(
    const std::shared_ptr<typename ServiceT::Request>/*request*/,
    const std::shared_ptr<typename ServiceT::Response>/*response*/) {}

  void handle_service(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<typename ServiceT::Request> request,
    const std::shared_ptr<typename ServiceT::Response> response)
  {
    request_count_++;
    fillResponse(request, response);
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Service<ServiceT>::SharedPtr service_;
  std::string service_name_;
  int request_count_;
  bool disabled_;
};

}  // namespace nav2_system_tests

#endif  // BEHAVIOR_TREE__DUMMY_SERVICE_HPP_
