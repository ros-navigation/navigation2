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

#ifndef NAV2_POSE_CLASSIFIERS__CLASSIFIER_BASE_HPP_
#define NAV2_POSE_CLASSIFIERS__CLASSIFIER_BASE_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_pose_classifiers
{

/**
 * @class ClassifierBase
 * @brief Abstract interface for pose classification plugins.
 *
 * Each plugin answers one question: "does this pose belong to my class?"
 * The path splitter in nav2_planner loads these via pluginlib and
 * iterates them in priority order per pose.
 */
class ClassifierBase
{
public:
  using Ptr = std::shared_ptr<ClassifierBase>;

  virtual ~ClassifierBase() {}

  /**
   * @brief Configure the classifier from ROS parameters.
   * @param parent   Weak pointer to the owning lifecycle node
   * @param name     Parameter namespace for this classifier instance
   * @param tf       TF buffer
   * @param costmap_ros  Shared costmap
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  /**
   * @brief Check if a pose belongs to this classifier's class.
   * @param pose  The pose to classify
   * @return true if the pose matches this class
   */
  virtual bool matches(const geometry_msgs::msg::PoseStamped & pose) = 0;

  /**
   * @brief Return the class_type enum value for this classifier.
   *        Must match a constant from PathClasses.msg.
   */
  virtual uint16_t classType() = 0;
};

}  // namespace nav2_pose_classifiers

#endif  // NAV2_POSE_CLASSIFIERS__CLASSIFIER_BASE_HPP_
