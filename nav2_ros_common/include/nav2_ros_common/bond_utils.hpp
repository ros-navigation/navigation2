// Copyright (c) 2026 Open Navigation LLC
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

#ifndef NAV2_ROS_COMMON__BOND_UTILS_HPP_
#define NAV2_ROS_COMMON__BOND_UTILS_HPP_

#include <string>

namespace nav2
{

/**
 * @brief Topic used for a lifecycle server's bond with the lifecycle manager.
 *
 * bondcpp historically used a single shared "bond" topic for every connection.
 * Each Bond creates its own subscription, so every heartbeat is delivered to
 * every Bond object and filtered in software (O(N^2) callbacks for N bonds).
 * Using a per-server topic keeps the bondcpp Status protocol while limiting
 * delivery to the two peers that need the message.
 *
 * Both LifecycleNode::createBond() and LifecycleManager::createBondConnection()
 * must use this helper so the topic names stay aligned.
 *
 * @param node_name Managed server name (same string used as the bond id)
 * @return Relative topic name for that bond pair
 */
// AI-assisted contribution for ros-navigation/navigation2#6061
inline std::string bond_topic_name(const std::string & node_name)
{
  return std::string("bond/") + node_name;
}

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__BOND_UTILS_HPP_
