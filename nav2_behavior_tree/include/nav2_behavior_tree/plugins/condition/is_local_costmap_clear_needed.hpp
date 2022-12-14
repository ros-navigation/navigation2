//
// Created by josh on 12/13/22.
//

#ifndef NAV2_WS_IS_LOCAL_COSTMAP_CLEAR_NEEDED_HPP
#define NAV2_WS_IS_LOCAL_COSTMAP_CLEAR_NEEDED_HPP

#include <string>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/plugins/condition/generic_error_code_condition.hpp"

namespace nav2_behavior_tree
{

class IsLocalCostmapClearNeeded : public GenericErrorCodeCondition
{
  using Action = nav2_msgs::action::FollowPath;
  using ActionGoal = Action::Goal;
public:
  IsLocalCostmapClearNeeded(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsLocalCostmapClearNeeded() = delete;
};

}
#endif //NAV2_WS_IS_LOCAL_COSTMAP_CLEAR_NEEDED_HPP
