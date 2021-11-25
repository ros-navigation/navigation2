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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REINITIALIZE_GLOBAL_LOCALIZATION_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REINITIALIZE_GLOBAL_LOCALIZATION_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_srvs/srv/empty.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::Empty
 */
class ReinitializeGlobalLocalizationService : public BtServiceNode<std_srvs::srv::Empty>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ReinitializeGlobalLocalizationService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ReinitializeGlobalLocalizationService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REINITIALIZE_GLOBAL_LOCALIZATION_SERVICE_HPP_
