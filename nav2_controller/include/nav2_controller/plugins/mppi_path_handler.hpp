// Copyright (c) 2025 Maurice Alexander Purnawan
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

#ifndef NAV2_CONTROLLER__PLUGINS__MPPI_PATH_HANDLER_HPP_
#define NAV2_CONTROLLER__PLUGINS__MPPI_PATH_HANDLER_HPP_

#include <string>
#include <vector>
#include "nav2_controller/plugins/simple_path_handler.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_controller
{
/**
* @class MPPIPathHandler
* @brief This plugin manages the global plan by clipping it to the local
* segment relevant to the controller—typically bounded by the local costmap size
* and transforming the resulting path into the costmap's global frame.
*/

class MPPIPathHandler : public SimplePathHandler
{
public:
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const rclcpp::Logger & logger,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

protected:
  /**
    * @brief Get global plan within window of the local costmap size
    * @param global_pose Robot pose
    * @return plan transformed in the costmap's global frame and iterator to the first pose of the global
    * plan (for pruning)
    */
  std::pair<nav_msgs::msg::Path, PathIterator> getGlobalPlanConsideringBoundsInCostmapFrame(
    const geometry_msgs::msg::PoseStamped & global_pose) override;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rclcpp::Logger logger_ {rclcpp::get_logger("Controller Server")};
  std::string plugin_name_;
  double prune_distance_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__MPPI_PATH_HANDLER_HPP_
