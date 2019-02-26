// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "nav2_controller/nav2_controller_client.hpp"

#include <memory>

namespace nav2_controller
{

Nav2ControllerClient::Nav2ControllerClient()
{
  node_ = std::make_shared<rclcpp::Node>("nav2_controller_client");

  request_ = std::make_shared<Srv::Request>();

  startup_client_  = node_->create_client<Srv>("startup");
  pause_client_    = node_->create_client<Srv>("pause");
  resume_client_   = node_->create_client<Srv>("resume");
  shutdown_client_ = node_->create_client<Srv>("shutdown");
}

void
Nav2ControllerClient::startup()
{
  callService(startup_client_, "startup");
}

void
Nav2ControllerClient::shutdown()
{
  callService(shutdown_client_, "shutdown");
}

void
Nav2ControllerClient::pause()
{
  callService(pause_client_, "pause");
}

void
Nav2ControllerClient::resume()
{
  callService(resume_client_, "resume");
}

void
Nav2ControllerClient::callService(rclcpp::Client<Srv>::SharedPtr service_client, const char * service_name)
{
  RCLCPP_INFO(node_->get_logger(), "Waiting for the nav2_controller's %s service...", service_name);
  while (!service_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Client interrupted while waiting for service to appear");
      return;
    }
	rclcpp::spin_some(node_);
    RCLCPP_INFO(node_->get_logger(), "Waiting for service to appear...");
  }

  RCLCPP_INFO(node_->get_logger(), "send_async_request (%s) to the nav2_controller", service_name);
  auto future_result = service_client->async_send_request(request_);

  rclcpp::executor::FutureReturnCode status = rclcpp::executor::FutureReturnCode::TIMEOUT;
  do {
    //RCLCPP_INFO(node_->get_logger(), "calling spin until future complete");
    status = rclcpp::spin_until_future_complete(node_, future_result, std::chrono::milliseconds(500));
  } while (rclcpp::ok() && status != rclcpp::executor::FutureReturnCode::SUCCESS);
}

}  // namespace nav2_controller
