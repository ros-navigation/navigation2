// Copyright (c) 2025 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONCATENATE_PATHS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONCATENATE_PATHS_ACTION_HPP_

#include <memory>
#include <vector>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp/action_node.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to concatenate two paths
 *
 * Both paths must be in the same frame. If their frames differ the node fails,
 * unless transform_to_path1_frame is set, in which case input_path2 is transformed
 * into the frame of input_path1 before the poses are appended.
 *
 * Usage in XML:
 * @code
 * <ConcatenatePaths input_path1="{main_path}" input_path2="{last_mile_path}" output_path="{path}"/>
 * @endcode
 */
class ConcatenatePaths : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::ConcatenatePaths constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ConcatenatePaths(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path1", "Input Path 1 to cancatenate"),
      BT::InputPort<nav_msgs::msg::Path>("input_path2", "Input Path 2 to cancatenate"),
      BT::InputPort<bool>(
        "transform_to_path1_frame", false,
        "Transform input_path2 into the frame of input_path1 when the frames differ, "
        "instead of failing"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Paths concatenated"),
    };
  }

private:
  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  nav2::LifecycleNode::SharedPtr node_;
  nav2::TransformBuffer::SharedPtr tf_;
  double transform_tolerance_{0.1};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONCATENATE_PATHS_ACTION_HPP_
