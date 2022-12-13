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

  /**
   * @brief overridable function that sets the current error codes
   */
  virtual void grabErrorCodesFromBT() = 0;

  /**
   * @brief overridable function that sets the error codes to check
   */
  virtual void setErrorCodesToCheck() = 0;

  BT::NodeStatus tick() override
  {
    setErrorCodesToCheck();
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

  static BT::PortsList providedPorts()
  {
    return {};
  }

protected:
  std::vector<unsigned int> current_error_codes_;
  std::vector<unsigned int> error_codes_to_check_;
};

}

#endif //NAV2_WS_GENERIC_ERROR_CODE_CONDITION_HPP
