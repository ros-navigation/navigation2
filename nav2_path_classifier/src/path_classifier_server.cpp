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

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"

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
  declare_parameter("publish_classified_paths", rclcpp::ParameterValue(false));
  declare_parameter(
    "default_class_type",
    rclcpp::ParameterValue(static_cast<int>(nav2_msgs::msg::ClassifiedPath::FREE_SPACE)));
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
  get_parameter("publish_classified_paths", publish_classified_paths_);
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

  // Configure path splitter (reads hysteresis/merge parameters)
  path_splitter_.configure(shared_from_this());

  if (publish_classified_paths_) {
    classified_segments_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("classified_plan_markers", 1);
    raw_classified_poses_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("raw_classified_poses_markers", 1);
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

  if (publish_classified_paths_) {
    classified_segments_marker_pub_->on_activate();
    raw_classified_poses_marker_pub_->on_activate();
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
  if (publish_classified_paths_) {
    classified_segments_marker_pub_->on_deactivate();
    raw_classified_poses_marker_pub_->on_deactivate();
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
  path_splitter_.cleanup();

  action_server_.reset();
  classified_segments_marker_pub_.reset();
  raw_classified_poses_marker_pub_.reset();
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

uint16_t PathClassifierServer::classify(
  const geometry_msgs::msg::PoseStamped & pose, bool fetch_data)
{
  // Priority order: first match wins
  for (auto & classifier : classifiers_) {
    if (classifier->matches(pose, fetch_data)) {
      return classifier->classType();
    }
  }
  return default_class_type_;
}

bool PathClassifierServer::hasClassifiers() const
{
  return !classifiers_.empty();
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

    if (hasClassifiers()) {
      auto split_result = path_splitter_.splitPath(
        goal->path, *this, publish_classified_paths_);
      result->classified_paths = split_result.classified_path_array;

      if (publish_classified_paths_) {
        classified_segments_marker_pub_->publish(
          buildSegmentMarkers(split_result.classified_path_array, goal->path.header));
        raw_classified_poses_marker_pub_->publish(
          buildRawPoseMarkers(split_result.classified_poses, goal->path.header));
      }
    } else {
      nav2_msgs::msg::ClassifiedPath cp;
      cp.class_type = default_class_type_;
      cp.path = goal->path;
      result->classified_paths.paths.push_back(cp);
    }

    result->classification_time = this->now() - start_time;
    action_server_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Failed to classify path: \"%s\"", ex.what());
    action_server_->terminate_current();
  }
}

/**
 * @brief Convert an HSV color to RGB.
 * @param hue Hue in [0, 1)
 * @param saturation Saturation in [0, 1]
 * @param value Brightness in [0, 1]
 * @return Equivalent RGB color with components in [0, 1]
 */
static std_msgs::msg::ColorRGBA hsvToRgb(double hue, double saturation, double value)
{
  const double h6 = hue * 6.0;
  const int sector = static_cast<int>(h6);
  const double f = h6 - sector;
  const double p = value * (1.0 - saturation);
  const double q = value * (1.0 - saturation * f);
  const double t = value * (1.0 - saturation * (1.0 - f));

  double r, g, b;
  switch (sector) {
    case 0: r = value; g = t; b = p; break;
    case 1: r = q; g = value; b = p; break;
    case 2: r = p; g = value; b = t; break;
    case 3: r = p; g = q; b = value; break;
    case 4: r = t; g = p; b = value; break;
    default: r = value; g = p; b = q; break;  // sector == 5
  }

  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = 1.0f;
  return color;
}

std_msgs::msg::ColorRGBA PathClassifierServer::colorForClass(uint16_t class_type)
{
  constexpr std::array<std::array<float, 3>, 6> kClassPalette = {{
    {{0.2f, 0.4f, 1.0f}},   // blue
    {{0.2f, 0.8f, 0.2f}},   // green
    {{0.9f, 0.1f, 0.1f}},   // red
    {{1.0f, 0.6f, 0.0f}},   // orange
    {{0.8f, 0.2f, 0.8f}},   // magenta
    {{0.2f, 0.8f, 0.8f}}    // cyan
  }};

  if (class_type < kClassPalette.size()) {
    const auto & c = kClassPalette[class_type];
    std_msgs::msg::ColorRGBA color;
    color.r = c[0];
    color.g = c[1];
    color.b = c[2];
    color.a = 1.0f;
    return color;
  }

  // Beyond the curated palette: step the hue by the golden ratio conjugate so
  // colors stay spread across the full hue circle no matter how many classes exist,
  // with no risk of repeating.
  constexpr double kGoldenRatioConjugate = 0.6180339887498949;
  constexpr double kSaturation = 0.85;
  constexpr double kValue = 0.95;
  const double hue = std::fmod(class_type * kGoldenRatioConjugate, 1.0);
  return hsvToRgb(hue, kSaturation, kValue);
}

visualization_msgs::msg::MarkerArray
PathClassifierServer::buildSegmentMarkers(
  const nav2_msgs::msg::ClassifiedPathArray & paths,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;

  // Delete previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.header = header;
  del.ns = kSegmentMarkerNs;
  markers.markers.push_back(del);

  for (size_t s = 0; s < paths.paths.size(); ++s) {
    const auto & seg = paths.paths[s];
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = kSegmentMarkerNs;
    m.id = static_cast<int>(s);
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = kSegmentLineWidth;
    m.color = colorForClass(seg.class_type);
    m.color.a = 0.9;
    m.pose.orientation.w = 1.0;
    for (const auto & pose : seg.path.poses) {
      m.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(m);
  }

  return markers;
}

visualization_msgs::msg::MarkerArray
PathClassifierServer::buildRawPoseMarkers(
  const std::vector<ClassifiedPose> & poses,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;

  // Delete previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.header = header;
  del.ns = kRawPoseMarkerNs;
  markers.markers.push_back(del);

  for (size_t i = 0; i < poses.size(); ++i) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = kRawPoseMarkerNs;
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = kRawPoseSize;
    m.scale.y = kRawPoseSize;
    m.scale.z = kRawPoseSize;
    m.color = colorForClass(poses[i].class_type);
    m.color.a = 0.8;
    m.pose = poses[i].pose.pose;
    markers.markers.push_back(m);
  }

  return markers;
}

}  // namespace nav2_path_classifier

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_path_classifier::PathClassifierServer)
