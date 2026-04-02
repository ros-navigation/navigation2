// Copyright (c) 2026 Origin
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_MANAGER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_MANAGER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/classified_path_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/msg/path_classes.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that manages a queue of classified path segments.
 *
 * On each tick it loads the front segment, maps its class_type to a controller_id
 * via a configurable vector, writes current_path / controller_id / local_goal to
 * output ports, and ticks the child. When the child succeeds, it pops the segment
 * and loads the next one. Returns SUCCESS when all segments are consumed.
 */
class PathManager : public BT::DecoratorNode
{
public:
  PathManager(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav2_msgs::msg::ClassifiedPathArray>(
        "path_array", "Classified path segments to follow"),
      BT::InputPort<nav2_msgs::msg::ClassifiedPathArray>(
        "replanned_path_array", "Reclassified sub-segments from a replan of the current segment"),
      BT::InputPort<std::vector<std::string>>(
        "path_class_to_controller_map", "Vector of controller IDs indexed by class_type"),
      BT::OutputPort<nav2_msgs::msg::ClassifiedPathArray>(
        "updated_path_array", "Remaining path segments after popping completed ones"),
      BT::OutputPort<nav_msgs::msg::Path>(
        "current_path", "Path of the current segment"),
      BT::OutputPort<std::string>(
        "controller_id", "Controller ID for the current segment"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "local_goal", "Last pose of the current segment path"),
    };
  }

private:
  BT::NodeStatus tick() override;

  /**
   * @brief Load the front segment into output ports
   * @return true if a segment was loaded, false if segments are empty
   */
  bool loadCurrentSegment();
  void publishSegmentMarkers();

  nav2_msgs::msg::ClassifiedPathArray segments_;
  std::vector<std::string> path_class_to_controller_map_;
  bool segment_loaded_;
  std_msgs::msg::Header last_replanned_header;  // to detect changes
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_array_pub_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_MANAGER_HPP_
