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

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"

#include "nav2_behavior_tree/plugins/action/classify_path_action.hpp"
#include "nav2_behavior_tree/utils/classified_path_markers.hpp"
#include "nav2_behavior_tree/utils/footprint_utils.hpp"

namespace nav2_behavior_tree
{

using nav2_msgs::msg::ClassifiedPose;
using nav2_util::geometry_utils::calculate_path_length;
using nav2_util::geometry_utils::euclidean_distance;

ClassifyPath::ClassifyPath(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  std::string costmap_topic = "global_costmap/costmap_raw";
  std::string footprint_topic = "global_costmap/published_footprint";
  bool publish_classified_path = false;
  getInput("costmap_topic", costmap_topic);
  getInput("footprint_topic", footprint_topic);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("use_confined_space", use_confined_space_);
  getInput("inflation_resolution", inflation_resolution_);
  getInput("max_constraint_clearance", max_constraint_clearance_);
  getInput("use_last_mile", use_last_mile_);
  getInput("last_mile_ratio", last_mile_ratio_);
  getInput("last_mile_distance", last_mile_distance_);
  getInput("use_classification_filter", use_classification_filter_);
  getInput("hysteresis_window", hysteresis_window_);
  getInput("publish_classified_path", publish_classified_path);

  // Only enabled steps are validated, as the server only validated loaded plugins
  if (use_confined_space_ && inflation_resolution_ <= 0.0) {
    throw std::runtime_error("ClassifyPath: 'inflation_resolution' must be positive");
  }
  if (use_last_mile_ && (last_mile_ratio_ < 0.0 || last_mile_ratio_ > 1.0)) {
    throw std::runtime_error("ClassifyPath: 'last_mile_ratio' must be in [0, 1]");
  }
  if (use_last_mile_ && last_mile_distance_ < 0.0) {
    throw std::runtime_error("ClassifyPath: 'last_mile_distance' must not be negative");
  }
  if (use_classification_filter_ && hysteresis_window_ < 1) {
    RCLCPP_WARN(
      node_->get_logger(), "ClassifyPath: hysteresis_window must be >= 1, clamping to 1");
    hysteresis_window_ = 1;
  }

  // Own callback group spun in tick(), as nothing spins the BT node
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {costmap_msg_ = msg;}, sub_option);
  footprint_sub_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic, rclcpp::SystemDefaultsQoS(),
    [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {footprint_msg_ = msg;},
    sub_option);

  if (publish_classified_path) {
    raw_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "raw_classified_path_markers", 1);
    if (use_classification_filter_) {
      filtered_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "filtered_classified_path_markers", 1);
    }
  }
}

BT::NodeStatus ClassifyPath::tick()
{
  callback_group_executor_.spin_some();

  const auto fail = [this]() {
      setOutput("classified_path", std::vector<ClassifiedPose>());
      return BT::NodeStatus::FAILURE;
    };

  nav_msgs::msg::Path path;
  getInput("path", path);
  if (path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "ClassifyPath: received an empty path");
    return fail();
  }

  // As in the server, no usable footprint skips confined space, while a missing costmap fails
  const bool classify_confined_space =
    use_confined_space_ && updateFootprint() && footprint_.size() >= 3;
  if (classify_confined_space) {
    if (!updateCostmap()) {
      RCLCPP_WARN(node_->get_logger(), "ClassifyPath: no costmap received yet");
      return fail();
    }
    if (path.header.frame_id != costmap_frame_) {
      RCLCPP_ERROR(
        node_->get_logger(), "ClassifyPath: path frame '%s' does not match costmap frame '%s'",
        path.header.frame_id.c_str(), costmap_frame_.c_str());
      return fail();
    }
  }

  std::vector<ClassifiedPose> classified_path(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    classified_path[i].pose = path.poses[i];
    classified_path[i].class_type = ClassifiedPose::FREE_SPACE;
  }

  // Call order sets priority, as each step only claims FREE_SPACE poses
  if (use_last_mile_) {
    classifyLastMile(path, classified_path);
  }
  if (classify_confined_space) {
    classifyConfinedSpace(path, classified_path);
  }

  if (raw_markers_pub_) {
    raw_markers_pub_->publish(utils::createRawPathMarkers(classified_path, path.header));
  }

  if (use_classification_filter_) {
    filterClasses(classified_path, hysteresis_window_);
    if (filtered_markers_pub_) {
      filtered_markers_pub_->publish(
        utils::createFilteredPathMarkers(classified_path, path.header));
    }
  }

  setOutput("classified_path", classified_path);
  return BT::NodeStatus::SUCCESS;
}

bool ClassifyPath::updateFootprint()
{
  // Converted on every tick like the server, so a transform failure skips confined space
  std::vector<geometry_msgs::msg::Point> footprint;
  if (!footprint_msg_ ||
    !utils::footprintToRobotFrame(
      *footprint_msg_, *tf_, robot_base_frame_, transform_tolerance_, footprint))
  {
    return false;
  }

  if (!utils::footprintChanged(footprint, footprint_)) {
    return true;
  }
  footprint_ = footprint;
  opposites_ = utils::buildOppositePairs(footprint_);

  // Inflations don't depend on the pose, so build them once per footprint
  const int max_steps = static_cast<int>(
    std::ceil(max_constraint_clearance_ / inflation_resolution_));
  inflated_footprints_.clear();
  bool all_inflated = true;
  for (int step = 1; step <= max_steps; ++step) {
    inflated_footprints_.push_back(
      utils::inflateFootprint(footprint_, step * inflation_resolution_));
    all_inflated = all_inflated && inflated_footprints_.back().size() == footprint_.size();
  }
  if (!all_inflated && footprint_.size() >= 3) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "ClassifyPath: footprint can't be inflated edge for edge, confined space is not detected");
  }
  return true;
}

bool ClassifyPath::updateCostmap()
{
  if (costmap_msg_) {
    const auto & meta = costmap_msg_->metadata;
    if (!costmap_) {
      costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
        meta.size_x, meta.size_y, meta.resolution,
        meta.origin.position.x, meta.origin.position.y);
      collision_checker_.setCostmap(costmap_.get());
    } else if (costmap_->getSizeInCellsX() != meta.size_x ||  // NOLINT
      costmap_->getSizeInCellsY() != meta.size_y ||
      costmap_->getResolution() != meta.resolution ||
      costmap_->getOriginX() != meta.origin.position.x ||
      costmap_->getOriginY() != meta.origin.position.y)
    {
      costmap_->resizeMap(
        meta.size_x, meta.size_y, meta.resolution,
        meta.origin.position.x, meta.origin.position.y);
    }
    std::copy_n(
      costmap_msg_->data.begin(), meta.size_x * meta.size_y, costmap_->getCharMap());
    costmap_frame_ = costmap_msg_->header.frame_id;
    costmap_msg_.reset();
  }
  return costmap_ != nullptr;
}

void ClassifyPath::classifyLastMile(
  const nav_msgs::msg::Path & path,
  std::vector<ClassifiedPose> & classified_path) const
{
  // Ratio tail or the last_mile_distance before the goal, whichever is longer
  const double path_length = calculate_path_length(path);
  const double last_mile_start = std::min(
    path_length * (1.0 - last_mile_ratio_),
    std::max(0.0, path_length - last_mile_distance_));

  // Summed in the same order as calculate_path_length so the goal lands exactly on path_length
  double dist_from_start = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    if (i > 0) {
      dist_from_start += euclidean_distance(path.poses[i - 1].pose, path.poses[i].pose);
    }
    if (dist_from_start >= last_mile_start &&
      classified_path[i].class_type == ClassifiedPose::FREE_SPACE)
    {
      classified_path[i].class_type = ClassifiedPose::LAST_MILE;
    }
  }
}

void ClassifyPath::classifyConfinedSpace(
  const nav_msgs::msg::Path & path,
  std::vector<ClassifiedPose> & classified_path)
{
  for (size_t i = 0; i < path.poses.size(); ++i) {
    if (classified_path[i].class_type == ClassifiedPose::FREE_SPACE &&
      isConfined(path.poses[i]))
    {
      classified_path[i].class_type = ClassifiedPose::CONFINED_SPACE;
    }
  }
}

bool ClassifyPath::isConfined(const geometry_msgs::msg::PoseStamped & pose)
{
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double theta = tf2::getYaw(pose.pose.orientation);
  const size_t n = footprint_.size();

  // Edges that have hit a lethal cell so far
  std::vector<bool> lethal_edges(n, false);

  for (const auto & inflated_fp : inflated_footprints_) {
    // Skip steps that couldn't be inflated edge for edge
    if (inflated_fp.size() != n) {
      continue;
    }

    std::vector<geometry_msgs::msg::Point> world_fp;
    nav2_costmap_2d::transformFootprint(x, y, theta, inflated_fp, world_fp);

    std::vector<std::pair<unsigned int, unsigned int>> cells(n);
    bool all_in_map = true;
    for (size_t i = 0; i < n; ++i) {
      if (!costmap_->worldToMap(
          world_fp[i].x, world_fp[i].y,
          cells[i].first, cells[i].second))
      {
        all_in_map = false;
        break;
      }
    }
    if (!all_in_map) {
      continue;  // some vertices outside map, skip this step
    }

    for (size_t i = 0; i < n; ++i) {
      const size_t j = (i + 1) % n;
      const double cost = collision_checker_.lineCost(
        cells[i].first, cells[j].first,
        cells[i].second, cells[j].second);

      if (cost == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
        lethal_edges[i] = true;
        if (opposites_[i] < n && lethal_edges[opposites_[i]]) {
          return true;  // both opposite edges are walled
        }
      }
    }
  }

  return false;
}

void ClassifyPath::filterClasses(
  std::vector<ClassifiedPose> & classified_path, int hysteresis_window)
{
  if (classified_path.empty()) {
    return;
  }
  hysteresis_window = std::max(hysteresis_window, 1);

  // No class is accepted until one is confirmed, so poses before it are treated as noise
  bool has_current = false;
  uint16_t current_class = 0;
  uint16_t pending_class = classified_path[0].class_type;
  int run_count = 0;

  for (size_t i = 0; i < classified_path.size(); ++i) {
    const uint16_t raw = classified_path[i].class_type;
    if (has_current && raw == current_class) {
      // Matches the accepted class, so drop any pending transition
      run_count = 0;
      pending_class = current_class;
    } else if (raw == pending_class) {
      ++run_count;
    } else {
      // A different class appeared, so start a new pending run
      pending_class = raw;
      run_count = 1;
    }

    if (run_count >= hysteresis_window) {
      // Class confirmed, backfill the pending run, or every pose so far if it's the first
      const size_t backfill_start =
        has_current ? i + 1 - static_cast<size_t>(run_count) : 0;
      current_class = pending_class;
      has_current = true;
      for (size_t j = backfill_start; j < i; ++j) {
        classified_path[j].class_type = current_class;
      }
      run_count = 0;
    }

    if (has_current) {
      classified_path[i].class_type = current_class;
    }
  }

  // No class was ever confirmed, so keep the first pose's class
  if (!has_current) {
    const uint16_t first_class = classified_path[0].class_type;
    for (auto & classified_pose : classified_path) {
      classified_pose.class_type = first_class;
    }
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClassifyPath>("ClassifyPath");
}
