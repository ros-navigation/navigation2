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
}

void IsLocalCostmapClearNeeded::grabErrorCodesFromBT()
{
  current_error_codes_.push_back(100);
}

void IsLocalCostmapClearNeeded::setErrorCodesToCheck()
{
  error_codes_to_check_.push_back(100);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsLocalCostmapClearNeeded>
        ("IsLocalCostmapClearNeeded");
}
