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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLASSIFY_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLASSIFY_PATH_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief Classifies each pose of a path
 */
class ClassifyPath : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ClassifyPath
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ClassifyPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to classify"),
      BT::InputPort<std::string>(
        "costmap_topic", std::string("global_costmap/costmap_raw"), "Costmap topic"),
      BT::InputPort<std::string>(
        "footprint_topic", std::string("global_costmap/published_footprint"),
        "Footprint topic"),
      BT::InputPort<std::string>(
        "robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<bool>("use_confined_space", true, "Classify confined space"),
      BT::InputPort<double>(
        "inflation_resolution", 0.05, "Footprint inflation per step in metres"),
      BT::InputPort<double>(
        "max_constraint_clearance", 0.1,
        "Footprint inflation at which walls on opposite edges mark confined space"),
      BT::InputPort<bool>("use_last_mile", true, "Classify last mile"),
      BT::InputPort<double>(
        "last_mile_ratio", 0.0, "Tail of the path length that is last mile"),
      BT::InputPort<double>(
        "last_mile_distance", 2.0, "Distance to the goal along the path that is last mile"),
      BT::InputPort<bool>(
        "use_classification_filter", true, "Filter classes with hysteresis"),
      BT::InputPort<int>(
        "hysteresis_window", 4, "Consecutive poses of a new class needed to confirm it"),
      BT::InputPort<bool>(
        "publish_classified_path", false, "Publish raw and classified path markers"),
      BT::OutputPort<std::vector<nav2_msgs::msg::ClassifiedPose>>(
        "classified_path", "Classified poses of the path"),
    };
  }

  /**
   * @brief Hysteresis filter over the classes of a classified path, in place
   * @param classified_path Classified poses in path order
   * @param hysteresis_window Poses of a new class needed to confirm it, below 1 is treated as 1
   */
  static void filterClasses(
    std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path, int hysteresis_window);

protected:
  /**
   * @brief Classify the input path and output its classified poses
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Convert the latest footprint to the robot frame, rebuilding pairs and inflations
   * @return false if no footprint was received or its transform failed
   */
  bool updateFootprint();

  /**
   * @brief Convert the latest costmap message into the costmap used for collision checks
   * @return true if a costmap is available
   */
  bool updateCostmap();

  /**
   * @brief Mark the FREE_SPACE poses in the tail of the path, by arc length, as LAST_MILE
   * @param path Path to classify
   * @param classified_path Classified poses of the path, updated in place
   */
  void classifyLastMile(
    const nav_msgs::msg::Path & path,
    std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path) const;

  /**
   * @brief Mark the confined FREE_SPACE poses as CONFINED_SPACE
   * @param path Path to classify
   * @param classified_path Classified poses of the path, updated in place
   */
  void classifyConfinedSpace(
    const nav_msgs::msg::Path & path,
    std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path);

  /**
   * @brief Whether walls hit opposite edges of the inflated footprint placed at the pose
   * @param pose Pose to check, in the costmap frame
   * @return true if the pose lies in confined space
   */
  bool isConfined(const geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_{0.1};
  std::string robot_base_frame_{"base_link"};

  bool use_confined_space_{true};
  double inflation_resolution_{0.05};
  double max_constraint_clearance_{0.1};
  bool use_last_mile_{true};
  double last_mile_ratio_{0.0};
  double last_mile_distance_{2.0};
  bool use_classification_filter_{true};
  int hysteresis_window_{4};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_msg_;
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_msg_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr classified_markers_pub_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::string costmap_frame_;
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_;

  std::vector<geometry_msgs::msg::Point> footprint_;
  std::vector<size_t> opposites_;
  std::vector<std::vector<geometry_msgs::msg::Point>> inflated_footprints_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CLASSIFY_PATH_ACTION_HPP_
