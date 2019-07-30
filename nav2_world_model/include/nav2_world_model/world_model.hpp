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

#ifndef NAV2_WORLD_MODEL__WORLD_MODEL_HPP_
#define NAV2_WORLD_MODEL__WORLD_MODEL_HPP_

#include <memory>
#include <thread>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

namespace nav2_world_model
{

class WorldModel : public nav2_util::LifecycleNode
{
public:
  WorldModel();
  ~WorldModel();

protected:
  // Implement the lifecycle interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  // The WorldModel provides these services
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_service_;

  void costmap_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response);

  // The implementation of the WorldModel uses a Costmap2DROS node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // The thread for the Costmap2DROS node
  std::unique_ptr<std::thread> costmap_thread_;

  // The frame_id and metadata layer values used in the service response message
  static constexpr const char * frame_id_{"map"};
  static constexpr const char * metadata_layer_{"Master"};

  // An executor used to spin the costmap node
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> costmap_executor_;
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__WORLD_MODEL_HPP_
