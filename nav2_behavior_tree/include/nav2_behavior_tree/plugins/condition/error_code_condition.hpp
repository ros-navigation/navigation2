// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ERROR_CODE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ERROR_CODE_CONDITION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace BT
{
template<> inline std::vector<int16_t> convertFromString(StringView str)
{
  // We expect real numbers separated by semicolons
  auto parts = splitString(str, ',');

  std::vector<int16_t> vec_int;
  for (const auto part : parts) {
    vec_int.push_back(convertFromString<int16_t>(part));
  }
  return vec_int;
}
}  // end namespace BT

namespace nav2_behavior_tree
{

class ErrorCodeCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ErrorCodeCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node connfiguration
   */
  ErrorCodeCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ErrorCodeCondition() = delete;

  BT::NodeStatus tick() override;
  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<int16_t>("error_code_id"),
        BT::InputPort<std::vector<int16_t>>("error_codes_to_check")
      };
  }

private:
  rclcpp::Node::SharedPtr node_;

  int16_t error_code_id_;
  std::vector<int16_t> error_codes_to_check_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ERROR_CODE_CONDITION_HPP_
