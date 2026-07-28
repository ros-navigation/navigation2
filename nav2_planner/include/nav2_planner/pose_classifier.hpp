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

#ifndef NAV2_PLANNER__POSE_CLASSIFIER_HPP_
#define NAV2_PLANNER__POSE_CLASSIFIER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_loader.hpp"
#include "nav2_pose_classifiers/classifier_base.hpp"
#include "nav2_msgs/msg/path_classes.hpp"

namespace nav2_planner
{

/**
 * @class PoseClassifier
 * @brief Loads ClassifierBase plugins via pluginlib and dispatches
 *        classify() calls in priority order (first match wins).
 *
 * Reads from the parent node's parameters:
 *   pose_classifier_plugins: ["ConstraintSpace"]
 *   ConstraintSpace:
 *     plugin: "nav2_pose_classifiers/ConstraintClassifier"
 *     inflation_resolution: 0.20
 *     max_constraint_clearance: 1.0
 */
class PoseClassifier
{
public:
  PoseClassifier();
  ~PoseClassifier() = default;

  /**
   * @brief Load and configure all classifier plugins from parameters.
   * @param parent  Lifecycle node that owns us (planner_server)
   * @param tf      TF buffer
   * @param costmap_ros  Shared costmap
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  void cleanup();
  void activate();
  void deactivate();

  /**
   * @brief Classify a single pose by iterating plugins in priority order.
   * @param pose  The pose to classify
   * @return class_type from the first matching plugin, or FREE_SPACE if none match
   */
  uint16_t classify(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Check if any classifier plugins are loaded.
   * @return true if at least one classifier plugin is configured
   */
  bool hasClassifiers() const;

private:
  pluginlib::ClassLoader<nav2_pose_classifiers::ClassifierBase> classifier_loader_;
  std::vector<nav2_pose_classifiers::ClassifierBase::Ptr> classifiers_;
  std::vector<std::string> classifier_ids_;
  std::vector<std::string> classifier_types_;
  rclcpp::Logger logger_{rclcpp::get_logger("PoseClassifier")};
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__POSE_CLASSIFIER_HPP_
