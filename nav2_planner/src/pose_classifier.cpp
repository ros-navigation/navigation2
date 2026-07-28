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

#include "nav2_planner/pose_classifier.hpp"

#include <stdexcept>

#include "nav2_util/node_utils.hpp"

namespace nav2_planner
{

PoseClassifier::PoseClassifier()
: classifier_loader_("nav2_pose_classifiers", "nav2_pose_classifiers::ClassifierBase")
{
}

void PoseClassifier::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PoseClassifier: parent node expired during configure");
  }
  logger_ = node->get_logger();

  // Read the list of classifier plugin names from parameters
  // Default: empty list (no classifiers → all poses are FREE_SPACE)
  nav2_util::declare_parameter_if_not_declared(
    node, "pose_classifier_plugins",
    rclcpp::ParameterValue(std::vector<std::string>{}));

  classifier_ids_ = node->get_parameter("pose_classifier_plugins").as_string_array();

  if (classifier_ids_.empty()) {
    RCLCPP_INFO(
      logger_, "PoseClassifier: no pose_classifier_plugins configured. "
      "All poses will be classified as FREE_SPACE.");
    return;
  }

  classifier_types_.resize(classifier_ids_.size());

  for (size_t i = 0; i < classifier_ids_.size(); ++i) {
    // Each classifier name has a ".plugin" param with the pluginlib type string
    classifier_types_[i] = nav2_util::get_plugin_type_param(node, classifier_ids_[i]);

    try {
      auto classifier = classifier_loader_.createSharedInstance(classifier_types_[i]);
      classifier->configure(parent, classifier_ids_[i], tf, costmap_ros);
      classifiers_.push_back(classifier);
      RCLCPP_INFO(
        logger_, "PoseClassifier: loaded classifier plugin '%s' of type '%s'",
        classifier_ids_[i].c_str(), classifier_types_[i].c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        logger_, "Failed to create classifier plugin '%s'. Exception: %s",
        classifier_ids_[i].c_str(), ex.what());
      throw;
    }
  }

  RCLCPP_INFO(
    logger_, "PoseClassifier: %zu classifier plugin(s) loaded.", classifiers_.size());
}

void PoseClassifier::cleanup()
{
  for (auto & classifier : classifiers_) {
    classifier->cleanup();
  }
  classifiers_.clear();
}

void PoseClassifier::activate()
{
  for (auto & classifier : classifiers_) {
    classifier->activate();
  }
}

void PoseClassifier::deactivate()
{
  for (auto & classifier : classifiers_) {
    classifier->deactivate();
  }
}

uint16_t PoseClassifier::classify(const geometry_msgs::msg::PoseStamped & pose)
{
  // Priority order: first match wins
  for (auto & classifier : classifiers_) {
    if (classifier->matches(pose)) {
      return classifier->classType();
    }
  }
  return nav2_msgs::msg::PathClasses::FREE_SPACE;
}

bool PoseClassifier::hasClassifiers() const
{
  return !classifiers_.empty();
}

}  // namespace nav2_planner
