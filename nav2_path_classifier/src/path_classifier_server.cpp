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
  pose_classifier_.configure(shared_from_this(), tf_, costmap_sub_, footprint_sub_);

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
  pose_classifier_.activate();
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
  pose_classifier_.deactivate();
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

  pose_classifier_.cleanup();
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

    if (pose_classifier_.hasClassifiers()) {
      auto split_result = path_splitter_.splitPath(
        goal->path, pose_classifier_, publish_classified_paths_);
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

std_msgs::msg::ColorRGBA PathClassifierServer::colorForClass(uint16_t class_type)
{
  constexpr float kShadeStep = 0.25f;
  constexpr float kShadeFloor = 0.40f;
  const size_t hue = class_type % kClassPalette.size();
  const size_t cycle = class_type / kClassPalette.size();
  float factor = 1.0f - kShadeStep * static_cast<float>(cycle);
  if (factor < kShadeFloor) {
    factor = kShadeFloor;
  }
  const auto & c = kClassPalette[hue];
  std_msgs::msg::ColorRGBA color;
  color.r = c[0] * factor;
  color.g = c[1] * factor;
  color.b = c[2] * factor;
  color.a = 1.0f;
  return color;
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
