#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CALL_TRIGGER_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CALL_TRIGGER_SERVICE_HPP_

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps std_srvs::srv::Trigger
 */
class CallTriggerService : public BtServiceNode<std_srvs::srv::Trigger>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::CallTriggerService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  CallTriggerService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CALL_TRIGGER_SERVICE_HPP_
