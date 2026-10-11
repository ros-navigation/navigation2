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

#ifndef NAV2_PATH_CLASSIFIER__PATH_CLASSIFIER_HPP_
#define NAV2_PATH_CLASSIFIER__PATH_CLASSIFIER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_path_classifier
{

/**
 * @class PathClassifier
 * @brief Abstract interface for path classification plugins.
 *
 * Each plugin classifies every pose of a path. The server runs the plugins in
 * priority order and merges their results, so a pose takes the class of the
 * first plugin that does not leave it UNKNOWN.
 */
class PathClassifier
{
public:
  using Ptr = std::shared_ptr<PathClassifier>;

  /**
   * @brief Virtual destructor
   */
  virtual ~PathClassifier() {}

  /**
   * @brief Configure the classifier from ROS parameters.
   * @param parent         Weak pointer to the owning lifecycle node
   * @param name           Parameter namespace for this classifier instance
   * @param tf             TF buffer
   * @param costmap_sub    Subscriber to the costmap the classifier reasons over
   * @param footprint_sub  Subscriber to the robot footprint
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate the classifier and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate the classifier and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Classify every pose of a path.
   * @param path Path to classify
   * @return One classified pose per path pose, UNKNOWN for poses this plugin does not classify
   */
  virtual std::vector<nav2_msgs::msg::ClassifiedPose> classify(
    const nav_msgs::msg::Path & path) = 0;
};

}  // namespace nav2_path_classifier

#endif  // NAV2_PATH_CLASSIFIER__PATH_CLASSIFIER_HPP_
