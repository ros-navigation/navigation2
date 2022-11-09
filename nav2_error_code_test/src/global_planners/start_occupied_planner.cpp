#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "global_planners/start_occupied_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"

namespace nav2_error_code_test
{

//nav_msgs::msg::Path StartOccupied::createPlan(
//  const geometry_msgs::msg::PoseStamped &,
//  const geometry_msgs::msg::PoseStamped &)
//{
//  throw nav2_core::StartOccupied("start occupied");
//}

}  // namespace nav2_error_code_test

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_error_code_test::StartOccupied, nav2_core::GlobalPlanner)
