//
// Created by josh on 12/13/22.
//

#include "nav2_behavior_tree/plugins/condition/is_local_costmap_clear_needed.hpp"
#include <memory>

namespace nav2_behavior_tree
{

IsLocalCostmapClearNeeded::IsLocalCostmapClearNeeded(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : GenericErrorCodeCondition(condition_name, conf)
{
  error_codes_to_check_ = {
      ActionGoal::UNKNOWN,
      ActionGoal::PATIENCE_EXCEEDED,
      ActionGoal::FAILED_TO_MAKE_PROGRESS,
      ActionGoal::NO_VALID_CONTROL
    };
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsLocalCostmapClearNeeded>
        ("IsLocalCostmapClearNeeded");
}
