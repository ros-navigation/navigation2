// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_

#include <memory>
#include <string>
#include <limits>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp/action_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class TruncatePathLocal : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TruncatePathLocal constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TruncatePathLocal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::OutputPort<nav_msgs::msg::Path>(
        "output_path", "Path truncated to a certain distance around robot"),
      BT::InputPort<double>(
        "distance_forward", 8.0,
        "Distance in forward direction"),
      BT::InputPort<double>(
        "distance_backward", 4.0,
        "Distance in backward direction"),
      BT::InputPort<std::string>(
        "robot_frame", "base_link",
        "Robot base frame id"),
      BT::InputPort<double>(
        "transform_tolerance", 0.2,
        "Transform lookup tolerance"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "pose", "Manually specified pose to be used"
        "if overriding current robot pose"),
      BT::InputPort<double>(
        "angular_distance_weight", 0.0,
        "Weight of angular distance relative to positional distance when finding which path "
        "pose is closest to robot. Not applicable on paths without orientations assigned"),
      BT::InputPort<double>(
        "max_robot_pose_search_dist", std::numeric_limits<double>::infinity(),
        "Maximum forward integrated distance along the path (starting from the last detected pose) "
        "to bound the search for the closest pose to the robot. When set to infinity (default), "
        "whole path is searched every time"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Get either specified input pose or robot pose in path frame
   * @param path_frame_id Frame ID of path
   * @param pose Output pose
   * @return True if succeeded
   */
  bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief A custom pose distance method which takes angular distance into account
   * in addition to spatial distance (to improve picking a correct pose near cusps and loops)
   * @param pose1 Distance is computed between this pose and pose2
   * @param pose2 Distance is computed between this pose and pose1
   * @param angular_distance_weight Weight of angular distance relative to spatial distance
   * (1.0 means that 1 radian of angular distance corresponds to 1 meter of spatial distance)
   */
  static double poseDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const geometry_msgs::msg::PoseStamped & pose2,
    const double angular_distance_weight);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path::_poses_type::iterator closest_pose_detection_begin_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_LOCAL_ACTION_HPP_
