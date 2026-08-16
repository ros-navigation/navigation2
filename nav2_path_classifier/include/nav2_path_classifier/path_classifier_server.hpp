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

#ifndef NAV2_PATH_CLASSIFIER__PATH_CLASSIFIER_SERVER_HPP_
#define NAV2_PATH_CLASSIFIER__PATH_CLASSIFIER_SERVER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/classify_path.hpp"
#include "nav2_msgs/msg/classified_path.hpp"
#include "nav2_msgs/msg/classified_path_array.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_path_classifier/pose_classifier.hpp"
#include "nav2_path_classifier/path_splitter.hpp"

namespace nav2_path_classifier
{

/**
 * @class nav2_path_classifier::PathClassifierServer
 * @brief Hosts ClassifierBase plugins and splits an incoming path into
 *        classified segments, exposed through the ClassifyPath action.
 */
class PathClassifierServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_path_classifier::PathClassifierServer
   * @param options Additional options to control creation of the node.
   */
  explicit PathClassifierServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PathClassifierServer() = default;

protected:
  /**
   * @brief Configure classifier plugins, splitter, subscribers and action server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate publishers, classifier plugins and the action server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the action server, classifier plugins and publishers
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::ClassifyPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  /**
   * @brief ClassifyPath action server callback. Classifies the goal path and
   *        returns it split into segments of uniform class.
   */
  void classifyPath();

  /**
   * @brief Build MarkerArray of line-strip segments from classified path array.
   */
  visualization_msgs::msg::MarkerArray buildSegmentMarkers(
    const nav2_msgs::msg::ClassifiedPathArray & paths,
    const std_msgs::msg::Header & header);

  /**
   * @brief Build MarkerArray of per-pose spheres from raw classified poses.
   */
  visualization_msgs::msg::MarkerArray buildRawPoseMarkers(
    const std::vector<ClassifiedPose> & poses,
    const std_msgs::msg::Header & header);

  // Our action server implements the ClassifyPath action
  std::unique_ptr<ActionServer> action_server_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Costmap and footprint sources shared with every classifier plugin
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  // Class assigned when no classifier plugin claims a pose
  uint16_t default_class_type_;

  // Marker visualization constants
  static constexpr const char * kSegmentMarkerNs = "classified_paths";
  static constexpr const char * kRawPoseMarkerNs = "raw_classified_poses";
  static constexpr double kSegmentLineWidth = 0.05;
  static constexpr double kRawPoseSize = 0.06;

  // Colors for path-class visualization markers.
  static constexpr std::array<std::array<float, 3>, 6> kClassPalette = {{
    {{0.2f, 0.4f, 1.0f}},   // blue    
    {{0.2f, 0.8f, 0.2f}},   // green
    {{0.9f, 0.1f, 0.1f}},    // red
    {{1.0f, 0.6f, 0.0f}},   // orange
    {{0.8f, 0.2f, 0.8f}},   // magenta
    {{0.2f, 0.8f, 0.8f}}   // cyan
  }};

  // Marker color for a given path class_type.
  static std_msgs::msg::ColorRGBA colorForClass(uint16_t class_type);

  // MarkerArray publishers for classified path visualization (enabled by publish_classified_paths)
  bool publish_classified_paths_{false};
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    classified_segments_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    raw_classified_poses_marker_pub_;

  // Pose classifier — loads ClassifierBase plugins, dispatches classify() per pose
  PoseClassifier pose_classifier_;

  // Path splitter — classifies path poses and splits into ClassifiedPathArray segments
  PathSplitter path_splitter_;
};

}  // namespace nav2_path_classifier

#endif  // NAV2_PATH_CLASSIFIER__PATH_CLASSIFIER_SERVER_HPP_
