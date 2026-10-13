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

#include "nav2_path_classifier/path_classifier_server.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "nav2_path_classifier/utils.hpp"

namespace nav2_path_classifier
{

PathClassifierServer::PathClassifierServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("path_classifier_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating path classifier server");

  declare_parameter(
    "costmap_topic",
    rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
  declare_parameter(
    "footprint_topic",
    rclcpp::ParameterValue(std::string("global_costmap/published_footprint")));
  declare_parameter(
    "robot_base_frame",
    rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter(
    "pose_classifier_plugins",
    rclcpp::ParameterValue(std::vector<std::string>{}));
  declare_parameter("publish_classified_path", rclcpp::ParameterValue(false));
  declare_parameter(
    "default_class_type",
    rclcpp::ParameterValue(static_cast<int>(nav2_msgs::msg::ClassifiedPose::FREE_SPACE)));
  declare_parameter("use_classification_filter", rclcpp::ParameterValue(true));
  declare_parameter("classification_filter.hysteresis_window", rclcpp::ParameterValue(4));
}

nav2_util::CallbackReturn
PathClassifierServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  std::string costmap_topic, footprint_topic, robot_base_frame;
  double transform_tolerance;
  get_parameter("costmap_topic", costmap_topic);
  get_parameter("footprint_topic", footprint_topic);
  get_parameter("robot_base_frame", robot_base_frame);
  get_parameter("transform_tolerance", transform_tolerance);
  get_parameter("publish_classified_path", publish_classified_path_);
  default_class_type_ = static_cast<uint16_t>(get_parameter("default_class_type").as_int());

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), costmap_topic);
  footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
    shared_from_this(), footprint_topic, *tf_, robot_base_frame, transform_tolerance);

  // Configure pose classifier plugins (if any specified in params)
  configureClassifiers();

  if (get_parameter("use_classification_filter").as_bool()) {
    classification_filter_ = std::make_unique<ClassificationFilter>(
      get_parameter("classification_filter.hysteresis_window").as_int(), get_logger());
  }

  if (publish_classified_path_) {
    raw_classified_path_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("raw_classified_path_markers", 1);
    if (classification_filter_) {
      filtered_classified_path_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
        "filtered_classified_path_markers", 1);
    }
  }

  // Create the action server that we implement with our classifyPath method
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "classify_path",
    std::bind(&PathClassifierServer::classifyPath, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PathClassifierServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (publish_classified_path_) {
    raw_classified_path_marker_pub_->on_activate();
  }
  if (filtered_classified_path_marker_pub_) {
    filtered_classified_path_marker_pub_->on_activate();
  }
  activateClassifiers();
  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PathClassifierServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  deactivateClassifiers();
  if (publish_classified_path_) {
    raw_classified_path_marker_pub_->on_deactivate();
  }
  if (filtered_classified_path_marker_pub_) {
    filtered_classified_path_marker_pub_->on_deactivate();
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PathClassifierServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cleanupClassifiers();
  classification_filter_.reset();

  action_server_.reset();
  raw_classified_path_marker_pub_.reset();
  filtered_classified_path_marker_pub_.reset();
  transform_listener_.reset();
  tf_.reset();
  footprint_sub_.reset();
  costmap_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PathClassifierServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void PathClassifierServer::configureClassifiers()
{
  classifier_ids_ = get_parameter("pose_classifier_plugins").as_string_array();

  if (classifier_ids_.empty()) {
    RCLCPP_INFO(
      get_logger(), "PathClassifierServer: no pose_classifier_plugins configured. "
      "All poses will be classified as the default class.");
    return;
  }

  classifier_types_.resize(classifier_ids_.size());

  for (size_t i = 0; i < classifier_ids_.size(); ++i) {
    // Each classifier name has a ".plugin" param with the pluginlib type string
    classifier_types_[i] = nav2_util::get_plugin_type_param(shared_from_this(), classifier_ids_[i]);

    try {
      auto classifier = classifier_loader_.createSharedInstance(classifier_types_[i]);
      classifier->configure(
        shared_from_this(), classifier_ids_[i], tf_, costmap_sub_, footprint_sub_);
      classifiers_.push_back(classifier);
      RCLCPP_INFO(
        get_logger(), "PathClassifierServer: loaded classifier plugin '%s' of type '%s'",
        classifier_ids_[i].c_str(), classifier_types_[i].c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create classifier plugin '%s'. Exception: %s",
        classifier_ids_[i].c_str(), ex.what());
      throw;
    }
  }

  RCLCPP_INFO(
    get_logger(), "PathClassifierServer: %zu classifier plugin(s) loaded.", classifiers_.size());
}

void PathClassifierServer::cleanupClassifiers()
{
  for (auto & classifier : classifiers_) {
    classifier->cleanup();
  }
  classifiers_.clear();
}

void PathClassifierServer::activateClassifiers()
{
  for (auto & classifier : classifiers_) {
    classifier->activate();
  }
}

void PathClassifierServer::deactivateClassifiers()
{
  for (auto & classifier : classifiers_) {
    classifier->deactivate();
  }
}

void PathClassifierServer::classifyPath()
{
  auto start_time = this->now();
  auto result = std::make_shared<Action::Result>();

  try {
    auto goal = action_server_->get_current_goal();
    if (!goal) {
      return;  // if action_server_ is inactive, goal would be a nullptr
    }

    if (action_server_->is_cancel_requested()) {
      action_server_->terminate_all();
      return;
    }

    if (goal->path.poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received an empty path to classify.");
      action_server_->terminate_current();
      return;
    }

    const auto & poses = goal->path.poses;

    // Run every classifier on the whole path
    std::vector<std::vector<nav2_msgs::msg::ClassifiedPose>> classifier_paths;
    classifier_paths.reserve(classifiers_.size());
    for (auto & classifier : classifiers_) {
      classifier_paths.push_back(classifier->classify(goal->path));
      if (classifier_paths.back().size() != poses.size()) {
        throw std::runtime_error("Classifier returned a different number of poses than the path");
      }
    }

    // Each pose takes the class of the first classifier, in priority order, that classified it
    auto & classified_path = result->classified_path;
    classified_path.resize(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      classified_path[i].pose = poses[i];
      classified_path[i].class_type = default_class_type_;
      for (const auto & classifier_path : classifier_paths) {
        if (classifier_path[i].class_type != nav2_msgs::msg::ClassifiedPose::UNKNOWN) {
          classified_path[i].class_type = classifier_path[i].class_type;
          break;
        }
      }
    }

    if (publish_classified_path_) {
      // Published before filtering, so these show the raw per-pose classes
      raw_classified_path_marker_pub_->publish(
        utils::createRawPathMarkers(classified_path, goal->path.header));
    }

    if (classification_filter_) {
      classification_filter_->filter(classified_path);
      if (publish_classified_path_) {
        filtered_classified_path_marker_pub_->publish(
          utils::createFilteredPathMarkers(classified_path, goal->path.header));
      }
    }

    result->classification_time = this->now() - start_time;
    action_server_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Failed to classify path: \"%s\"", ex.what());
    action_server_->terminate_current();
  }
}

}  // namespace nav2_path_classifier

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_path_classifier::PathClassifierServer)
