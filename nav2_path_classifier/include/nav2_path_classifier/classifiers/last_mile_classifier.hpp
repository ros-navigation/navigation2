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

#ifndef NAV2_PATH_CLASSIFIER__CLASSIFIERS__LAST_MILE_CLASSIFIER_HPP_
#define NAV2_PATH_CLASSIFIER__CLASSIFIERS__LAST_MILE_CLASSIFIER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_path_classifier/path_classifier.hpp"

namespace nav2_path_classifier
{

/**
 * @class LastMileClassifier
 * @brief Classifies the last part of a path, by arc length, as last mile.
 */
class LastMileClassifier : public PathClassifier
{
public:
  /**
   * @brief A constructor for nav2_path_classifier::LastMileClassifier
   */
  LastMileClassifier() = default;

  /**
   * @brief A destructor for nav2_path_classifier::LastMileClassifier
   */
  ~LastMileClassifier() override = default;

  /**
   * @brief Configure the classifier from ROS parameters.
   * @param parent Weak pointer to the owning lifecycle node
   * @param name Parameter namespace for this classifier instance
   * @param tf TF buffer
   * @param costmap_sub Costmap subscriber
   * @param footprint_sub Footprint subscriber
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  /**
   * @brief Classify every pose of a path as last mile or UNKNOWN.
   * @param path Path to classify
   * @return One classified pose per path pose
   */
  std::vector<nav2_msgs::msg::ClassifiedPose> classify(
    const nav_msgs::msg::Path & path) override;

protected:
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("LastMileClassifier")};

  uint16_t class_type_;
  double last_mile_ratio_;
  double claim_full_path_threshold_;
};

}  // namespace nav2_path_classifier

#endif  // NAV2_PATH_CLASSIFIER__CLASSIFIERS__LAST_MILE_CLASSIFIER_HPP_
