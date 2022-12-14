//
// Created by josh on 12/13/22.
//

#ifndef NAV2_WS_GENERIC_ERROR_CODE_CONDITION_HPP
#define NAV2_WS_GENERIC_ERROR_CODE_CONDITION_HPP

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace BT
{
template<> inline std::vector<int> convertFromString(StringView str)
{
  // We expect real numbers separated by commas
  auto parts = splitString(str, ',');

  std::vector<int> vec_int;
  for (const auto part : parts) {
    vec_int.push_back(convertFromString<int>(part));
  }
  return vec_int;
}
}  // end namespace BT

namespace nav2_behavior_tree
{

class GenericErrorCodeCondition : public BT::ConditionNode
{
public:
  GenericErrorCodeCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : BT::ConditionNode(condition_name, conf)
  {}

  GenericErrorCodeCondition() = delete;

  BT::NodeStatus tick() override
  {
    getInput<std::vector<int>>("current_error_codes", current_error_codes_);

    for (const auto & current_error_code : current_error_codes_) {
      for (const auto error_code_to_check : error_codes_to_check_) {
        if (current_error_code == error_code_to_check) {
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return
    {
        BT::InputPort<std::vector<int>>("current_error_codes")
    };
  }

protected:
  std::vector<int> current_error_codes_;
  std::vector<int> error_codes_to_check_;
};

}

#endif //NAV2_WS_GENERIC_ERROR_CODE_CONDITION_HPP
