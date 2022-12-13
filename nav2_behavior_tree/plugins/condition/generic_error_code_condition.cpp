//
// Created by josh on 12/13/22.
//


#include "nav2_behavior_tree/plugins/condition/generic_error_code_condition.hpp"
#include <memory>

namespace nav2_behavior_tree
{

GenericErrorCodeCondition::GenericErrorCodeCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : BT::ConditionNode(condition_name, conf)
{
//  setErrorCodesToCheck();
}

BT::NodeStatus GenericErrorCodeCondition::tick()
{
  grabErrorCodesFromBT();

  for (const auto & current_error_code : current_error_codes_) {
    for (const auto error_code_to_check : error_codes_to_check_) {
      if (current_error_code == error_code_to_check) {
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree
