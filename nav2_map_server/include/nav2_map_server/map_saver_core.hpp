// Copyright (c) 2020 Shivam Pandey pandeyshivam2017robotics@gmail.com
// Copyright (c) 2020 Samsung Research Russia
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

#ifndef NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_MAP_SAVER_CORE_HPP_
#define NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_MAP_SAVER_CORE_HPP_

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_map_server{

template <class mapT>
class MapSaver : public nav2_util::LifecycleNode
{
 public:
  /**
   * @brief A constructor for nav2_map_server::MapServer
   */
  MapSaver();

  /**
   * @brief A Destructor for nav2_map_server::MapServer
   */
  ~MapSaver() override;

 protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Start publishing the map using the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Stops publishing the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets the member variables
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
  /**
   * @brief Called when Error is raised
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

};
}  // namespace nav2_map_server
#endif //NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_MAP_SAVER_CORE_HPP_
