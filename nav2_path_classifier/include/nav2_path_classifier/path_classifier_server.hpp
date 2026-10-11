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

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/classify_path.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_path_classifier/classification_filter.hpp"
#include "nav2_path_classifier/path_classifier.hpp"

namespace nav2_path_classifier
{

/**
 * @class nav2_path_classifier::PathClassifierServer
 * @brief Hosts PathClassifier plugins and merges their per-pose classes by priority.
 */
class PathClassifierServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_path_classifier::PathClassifierServer
   * @param options Additional options to control creation of the node.
   */
  explicit PathClassifierServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for nav2_path_classifier::PathClassifierServer
   */
  ~PathClassifierServer() = default;

protected:
  /**
   * @brief Configure classifier plugins, classification filter, subscribers and action server
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

  /**
   * @brief Load and configure all classifier plugins from parameters.
   */
  void configureClassifiers();

  /**
   * @brief Cleanup all loaded classifier plugins.
   */
  void cleanupClassifiers();

  /**
   * @brief Activate all loaded classifier plugins.
   */
  void activateClassifiers();

  /**
   * @brief Deactivate all loaded classifier plugins.
   */
  void deactivateClassifiers();

  using Action = nav2_msgs::action::ClassifyPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  /**
   * @brief ClassifyPath action server callback. Classifies each pose of the goal path.
   */
  void classifyPath();

  std::unique_ptr<ActionServer> action_server_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  uint16_t default_class_type_;

  bool publish_classified_path_{false};
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    filtered_classified_path_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    raw_classified_path_marker_pub_;

  pluginlib::ClassLoader<nav2_path_classifier::PathClassifier> classifier_loader_{
    "nav2_path_classifier", "nav2_path_classifier::PathClassifier"};
  std::vector<nav2_path_classifier::PathClassifier::Ptr> classifiers_;
  std::vector<std::string> classifier_ids_;
  std::vector<std::string> classifier_types_;

  std::unique_ptr<ClassificationFilter> classification_filter_;
};

}  // namespace nav2_path_classifier

#endif  // NAV2_PATH_CLASSIFIER__PATH_CLASSIFIER_SERVER_HPP_
