// Copyright (c) 2020 Samsung Research Russia
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

#ifndef NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
#define NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace nav2_map_server
{

class CostmapFilterInfoServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_map_server::CostmapFilterInfoServer
   */
  CostmapFilterInfoServer();
  /**
   * @brief Destructor for the nav2_map_server::CostmapFilterInfoServer
   */
  ~CostmapFilterInfoServer();

protected:
  /**
   * @brief Creates CostmapFilterInfo publisher and forms published message from ROS parameters
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Publishes a CostmapFilterInfo message
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;

  std::unique_ptr<nav2_msgs::msg::CostmapFilterInfo> msg_;
};  // CostmapFilterInfoServer

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
