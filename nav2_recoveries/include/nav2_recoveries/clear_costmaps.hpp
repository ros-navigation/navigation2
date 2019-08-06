// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_RECOVERIES__CLEAR_COSTMAPS_HPP_
#define NAV2_RECOVERIES__CLEAR_COSTMAPS_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace nav2_recoveries
{
using nav2_msgs::srv::ClearEntireCostmap;

class ClearCostmaps
{
public:
  explicit ClearCostmaps(
    rclcpp::Node::SharedPtr & node,
    const std::string & srv_name = "ClearCostmaps");
  ~ClearCostmaps();

  void onCall(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<ClearEntireCostmap::Request>/*request*/,
    const std::shared_ptr<ClearEntireCostmap::Response>/*response*/);

protected:
  rclcpp::Node::SharedPtr node_;

  std::vector<std::string> service_names_;
  std::string services_string_list_;
  std::vector<rclcpp::Client<ClearEntireCostmap>::SharedPtr> costmap_clearing_services_;
  rclcpp::Service<ClearEntireCostmap>::SharedPtr bt_clear_costmap_servicer_;

  std::chrono::system_clock::time_point last_clear_time_;
  std::chrono::milliseconds service_timeout_in_s_;
  std::chrono::seconds frequency_timeout_in_s_;
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__CLEAR_COSTMAPS_HPP_
