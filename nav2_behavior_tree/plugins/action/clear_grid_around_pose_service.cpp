#include "nav2_behavior_tree/plugins/action/clear_grid_around_pose_service.hpp"

namespace nav2_behavior_tree
{

ClearGridAroundPoseService::ClearGridAroundPoseService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<nav2_msgs::srv::ClearGridAroundPose>(service_node_name, conf)
{
}

void ClearGridAroundPoseService::on_tick()
{
  getInput("reset_distance", request_->reset_distance);
  getInput("pose", request_->pose);
  increment_recovery_count();
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearGridAroundPoseService>("ClearGridAroundPose");
}
