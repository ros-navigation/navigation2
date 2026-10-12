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

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_path_classifier/classifiers/last_mile_classifier.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_path_classifier
{

using nav2_util::geometry_utils::calculate_path_length;
using nav2_util::geometry_utils::euclidean_distance;

void LastMileClassifier::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/*costmap_sub*/,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  name_ = name;

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("LastMileClassifier: parent node expired during configure");
  }
  logger_ = node->get_logger();

  // Required: every classifier instance must declare the class id it emits
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".class_type", rclcpp::ParameterType::PARAMETER_INTEGER);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".last_mile_ratio", rclcpp::ParameterValue(0.2));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".claim_full_path_threshold",
    rclcpp::ParameterValue(2.0));  // metres

  class_type_ = static_cast<uint16_t>(
    node->get_parameter(name_ + ".class_type").as_int());
  last_mile_ratio_ = node->get_parameter(name_ + ".last_mile_ratio").as_double();
  if (last_mile_ratio_ < 0.0 || last_mile_ratio_ > 1.0) {
    throw std::runtime_error(
            "LastMileClassifier: 'last_mile_ratio' parameter must be in [0, 1].");
  }
  claim_full_path_threshold_ =
    node->get_parameter(name_ + ".claim_full_path_threshold").as_double();
  if (claim_full_path_threshold_ < 0.0) {
    throw std::runtime_error(
            "LastMileClassifier: 'claim_full_path_threshold' parameter must not be negative.");
  }

  RCLCPP_INFO(
    logger_,
    "LastMileClassifier [%s] configured: last_mile_ratio=%.3f, "
    "claim_full_path_threshold=%.3f",
    name_.c_str(), last_mile_ratio_, claim_full_path_threshold_);
}

std::vector<nav2_msgs::msg::ClassifiedPose> LastMileClassifier::classify(
  const nav_msgs::msg::Path & path)
{
  std::vector<nav2_msgs::msg::ClassifiedPose> classified_path(path.poses.size());

  const double path_length = calculate_path_length(path);
  const double last_mile_start = path_length < claim_full_path_threshold_ ?
    0.0 : path_length * (1.0 - last_mile_ratio_);

  // Summed in the same order as calculate_path_length so the goal lands exactly on path_length
  double dist_from_start = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    classified_path[i].pose = path.poses[i];
    if (i > 0) {
      dist_from_start += euclidean_distance(path.poses[i - 1].pose, path.poses[i].pose);
    }
    if (dist_from_start >= last_mile_start) {
      classified_path[i].class_type = class_type_;
    }
  }
  return classified_path;
}

}  // namespace nav2_path_classifier

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_path_classifier::LastMileClassifier,
  nav2_path_classifier::PathClassifier)
