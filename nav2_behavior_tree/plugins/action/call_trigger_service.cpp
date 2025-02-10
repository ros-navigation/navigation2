#include "nav2_behavior_tree/plugins/action/call_trigger_service.hpp"

namespace nav2_behavior_tree
{

CallTriggerService::CallTriggerService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<std_srvs::srv::Trigger>(service_node_name, conf)
{
}

void CallTriggerService::on_tick()
{
  increment_recovery_count();
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CallTriggerService>("CallTriggerService");
}
