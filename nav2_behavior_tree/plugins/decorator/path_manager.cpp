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

#include <string>
#include <vector>

#include "nav2_behavior_tree/plugins/decorator/path_manager.hpp"

namespace nav2_behavior_tree
{

PathManager::PathManager(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  segment_loaded_(false)
{
}

BT::NodeStatus PathManager::tick()
{
  // On first tick, read controller map, create publisher, sync stale replanned data
  if (status() == BT::NodeStatus::IDLE) {
    if (!getInput("path_class_to_controller_map", path_class_to_controller_map_)) {
      throw BT::RuntimeError("PathManager: missing required input [path_class_to_controller_map]");
    }
    // Optional: if not provided, goal_checker_id is left unset (server uses its default)
    getInput("path_class_to_goal_checker_map", path_class_to_goal_checker_map_);
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    bool pub_path_array{false};
    if (node->has_parameter("pub_path_array")) {
      node->get_parameter("pub_path_array", pub_path_array);
    }
    if (!path_array_pub_ && pub_path_array) {
      rclcpp::QoS qos(1);
      qos.transient_local();
      path_array_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "path_manager/current_path_array", qos);
    }
    last_replanned_header = std_msgs::msg::Header();
    nav2_msgs::msg::ClassifiedPathArray stale;
    if (getInput("replanned_path_array", stale) && !stale.paths.empty()) {
      last_replanned_header = stale.paths[0].path.header;
    }
  }

  // Read path_array every tick — if it changed, reload segments
  nav2_msgs::msg::ClassifiedPathArray incoming;
  if (!getInput("path_array", incoming)) {
    throw BT::RuntimeError("PathManager: missing required input [path_array]");
  }
  if (incoming != segments_) {
    segments_ = incoming;
    segment_loaded_ = false;

    RCLCPP_INFO(
      rclcpp::get_logger("PathManager"),
      "New path_array loaded: %zu segments", segments_.paths.size());
    for (size_t i = 0; i < segments_.paths.size(); ++i) {
      const auto & seg = segments_.paths[i];
      RCLCPP_INFO(
        rclcpp::get_logger("PathManager"),
        "  Segment[%zu]: class_type=%d, poses=%zu, first=(%.2f,%.2f), last=(%.2f,%.2f)",
        i, seg.class_type, seg.path.poses.size(),
        seg.path.poses.empty() ? 0.0 : seg.path.poses.front().pose.position.x,
        seg.path.poses.empty() ? 0.0 : seg.path.poses.front().pose.position.y,
        seg.path.poses.empty() ? 0.0 : seg.path.poses.back().pose.position.x,
        seg.path.poses.empty() ? 0.0 : seg.path.poses.back().pose.position.y);
    }
    publishSegmentMarkers();
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Check if the subtree wrote new replanned_path_array (by comparing stamp)
  nav2_msgs::msg::ClassifiedPathArray replanned;
  if (getInput("replanned_path_array", replanned) &&
    !replanned.paths.empty() &&
    (replanned.paths[0].path.header != last_replanned_header))
  {
    last_replanned_header = replanned.paths[0].path.header;

    // Replace current segment with the replanned sub-segments
    segments_.paths.erase(segments_.paths.begin());
    segments_.paths.insert(
      segments_.paths.begin(),
      replanned.paths.begin(),
      replanned.paths.end());
    setOutput("updated_path_array", segments_);
    publishSegmentMarkers();
    segment_loaded_ = false;
  }

  // If no segment is loaded, load the front one
  if (!segment_loaded_) {
    if (!loadCurrentSegment()) {
      // No segments to process — we're done
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Tick the child (e.g. FollowPath subtree)
  const BT::NodeStatus child_status = child_node_->executeTick();

  switch (child_status) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      // Pop the completed segment
      segments_.paths.erase(segments_.paths.begin());
      setOutput("updated_path_array", segments_);
      publishSegmentMarkers();
      segment_loaded_ = false;

      // If more segments remain, load next and keep running
      if (!segments_.paths.empty()) {
        haltChild();
        loadCurrentSegment();
        return BT::NodeStatus::RUNNING;
      }
      // All segments done
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE:
    default:
      return BT::NodeStatus::FAILURE;
  }
}

bool PathManager::loadCurrentSegment()
{
  if (segments_.paths.empty()) {
    return false;
  }

  const auto & segment = segments_.paths.front();

  // Map class_type to controller_id
  if (segment.class_type >= path_class_to_controller_map_.size()) {
    throw BT::RuntimeError(
            "PathManager: class_type " + std::to_string(segment.class_type) +
            " has no mapping in class_controller_map (size=" +
            std::to_string(path_class_to_controller_map_.size()) + ")");
  }

  setOutput("current_path", segment.path);
  setOutput("controller_id", path_class_to_controller_map_[segment.class_type]);

  if (!path_class_to_goal_checker_map_.empty()) {
    if (segment.class_type >= path_class_to_goal_checker_map_.size()) {
      throw BT::RuntimeError(
              "PathManager: class_type " + std::to_string(segment.class_type) +
              " has no mapping in path_class_to_goal_checker_map (size=" +
              std::to_string(path_class_to_goal_checker_map_.size()) + ")");
    }
    setOutput("goal_checker_id", path_class_to_goal_checker_map_[segment.class_type]);
  }

  // local_goal = last pose of the segment's path
  if (!segment.path.poses.empty()) {
    auto local_goal = segment.path.poses.back();
    local_goal.header = segment.path.header;
    setOutput("local_goal", local_goal);
    const std::string gc_id =
      (!path_class_to_goal_checker_map_.empty() &&
       segment.class_type < path_class_to_goal_checker_map_.size())
      ? path_class_to_goal_checker_map_[segment.class_type] : "(default)";
    RCLCPP_INFO(
      rclcpp::get_logger("PathManager"),
      "Loaded segment: class_type=%d, controller=%s, goal_checker=%s, poses=%zu, "
      "local_goal=(%.2f, %.2f), frame=%s, segments_remaining=%zu",
      segment.class_type,
      path_class_to_controller_map_[segment.class_type].c_str(),
      gc_id.c_str(),
      segment.path.poses.size(),
      local_goal.pose.position.x, local_goal.pose.position.y,
      local_goal.header.frame_id.c_str(),
      segments_.paths.size());
  }

  segment_loaded_ = true;
  return true;
}

void PathManager::publishSegmentMarkers()
{
  if (!path_array_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  // Delete previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.ns = "path_manager_segments";
  if (!segments_.paths.empty() && !segments_.paths[0].path.poses.empty()) {
    del.header = segments_.paths[0].path.header;
  }
  markers.markers.push_back(del);

  for (size_t s = 0; s < segments_.paths.size(); ++s) {
    const auto & seg = segments_.paths[s];
    visualization_msgs::msg::Marker m;
    if (!seg.path.poses.empty()) {
      m.header = seg.path.header;
    }
    m.ns = "path_manager_segments";
    m.id = static_cast<int>(s);
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05;
    m.color.a = 0.9;
    if (seg.class_type == nav2_msgs::msg::PathClasses::CONSTRAINT_SPACE) {
      m.color.b = 1.0;  // blue
    } else {
      m.color.g = 1.0;  // green
    }
    m.pose.orientation.w = 1.0;
    for (const auto & pose : seg.path.poses) {
      m.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(m);
  }

  path_array_pub_->publish(markers);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathManager>("PathManager");
}
