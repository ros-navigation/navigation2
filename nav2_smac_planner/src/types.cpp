#include <smac_planner/types.hpp>
#include "geometry_msgs/msg/PoseStamped.hpp"
#include <smac_planner/utils.hpp>

namespace smac_planner
{

void SearchInfo::setStart(const geometry_msgs::msg::Point& start){
  _start_pose = start;
  is_start_behind_goal.reset();
}

geometry_msgs::msg::Pose SearchInfo::getSearchBound(){
  return _search_bound;
}

void SearchInfo::setSearchBound(const geometry_msgs::msg::Pose& search_bound){
  _search_bound = search_bound;
  is_start_behind_goal.reset();
}

bool SearchInfo::isStartBehindSearchBounds(){

  if (is_start_behind_goal){
    return *is_start_behind_goal;
  }
  else{
    is_start_behind_goal = smac_planner::Utils::isBehindPose(_start_pose, _search_bound);
    return *is_start_behind_goal;
  }
}

}  // namespace smac_planner