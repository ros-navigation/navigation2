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
// limitations under the License.

#ifndef UTILS__TEST_SERVICE_HPP_
#define UTILS__TEST_SERVICE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

template<class ServiceT>
class TestService : public rclcpp::Node
{
public:
  explicit TestService(
    std::string service_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_service", options)
  {
    using namespace std::placeholders;  // NOLINT

    server_ = create_service<ServiceT>(
      service_name,
      std::bind(&TestService::handle_service, this, _1, _2, _3));
  }

  std::shared_ptr<typename ServiceT::Request> getCurrentRequest() const
  {
    return current_request_;
  }

protected:
  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<typename ServiceT::Request> request,
    const std::shared_ptr<typename ServiceT::Response> response)
  {
    (void)request_header;
    current_request_ = request;
    setSuccessIfExists(response);
  }

private:
  template<typename T>
  auto setSuccessIfExists(std::shared_ptr<T> response) -> decltype(response->success, void())
  {
    response->success = true;
  }
  
  template<typename T>
  void setSuccessIfExists(T)
  {
  }

  typename rclcpp::Service<ServiceT>::SharedPtr server_;
  std::shared_ptr<typename ServiceT::Request> current_request_;
};

#endif  // UTILS__TEST_SERVICE_HPP_
