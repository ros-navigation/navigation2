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

#include "nav2_planner/path_splitter.hpp"

#include <algorithm>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/msg/path_classes.hpp"

namespace nav2_planner
{

// ---------------------------------------------------------------------------
// configure / cleanup
// ---------------------------------------------------------------------------

void PathSplitter::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PathSplitter: parent node expired during configure");
  }
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, "path_splitter.hysteresis_window",
    rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(
    node, "path_splitter.min_segment_poses",
    rclcpp::ParameterValue(5));

  hysteresis_window_ = node->get_parameter("path_splitter.hysteresis_window").as_int();
  min_segment_poses_ = node->get_parameter("path_splitter.min_segment_poses").as_int();

  if (hysteresis_window_ < 1) {
    RCLCPP_WARN(logger_, "PathSplitter: hysteresis_window must be >= 1, clamping to 1");
    hysteresis_window_ = 1;
  }
  if (min_segment_poses_ < 1) {
    RCLCPP_WARN(logger_, "PathSplitter: min_segment_poses must be >= 1, clamping to 1");
    min_segment_poses_ = 1;
  }

  RCLCPP_DEBUG(
    logger_,
    "PathSplitter configured: hysteresis_window=%d, min_segment_poses=%d",
    hysteresis_window_, min_segment_poses_);
}

void PathSplitter::cleanup() {}

// ---------------------------------------------------------------------------
// splitPath — orchestrates the full pipeline
// ---------------------------------------------------------------------------

PathSplitter::SplitResult PathSplitter::splitPath(
  const nav_msgs::msg::Path & path,
  PoseClassifier & pose_classifier)
{
  SplitResult result;

  if (path.poses.empty()) {
    return result;
  }

  // Stage 1: classify every pose
  auto raw_classes = classifyAllPoses(path, pose_classifier);

  // Build raw classified poses for visualization/debugging
  if (node->get_parameter("publish_classified_paths").as_bool()) {
    result.classified_poses.reserve(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
      nav2_msgs::msg::ClassifiedPose cp;
      cp.pose = path.poses[i];
      cp.class_type = raw_classes[i];
      result.classified_poses.push_back(cp);
    }
  }

  // Stage 2: hysteresis filter
  auto filtered_classes = applyHysteresis(raw_classes);

  // Stage 3: group consecutive same-type poses into segments
  auto segments = groupSegments(filtered_classes);

  // Stage 4: merge segments shorter than min_segment_poses
  mergeShortSegments(segments);

  // Stage 5: build ClassifiedPathArray with 1-pose overlap at boundaries
  result.classified_path_array = buildResult(path, segments);

  RCLCPP_DEBUG(
    logger_, "PathSplitter: %zu poses → %zu segments",
    path.poses.size(), result.classified_path_array.paths.size());

  return result;
}

// ---------------------------------------------------------------------------
// Stage 1: classify each pose
// ---------------------------------------------------------------------------

std::vector<uint16_t> PathSplitter::classifyAllPoses(
  const nav_msgs::msg::Path & path,
  PoseClassifier & pose_classifier)
{
  std::vector<uint16_t> classes;
  classes.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    classes.push_back(pose_classifier.classify(pose));
  }
  return classes;
}

// ---------------------------------------------------------------------------
// Stage 2: hysteresis filter
// ---------------------------------------------------------------------------

std::vector<uint16_t> PathSplitter::applyHysteresis(
  const std::vector<uint16_t> & raw_classes)
{
  const size_t n = raw_classes.size();
  std::vector<uint16_t> filtered(n);

  uint16_t current_class = raw_classes[0];
  uint16_t pending_class = current_class;
  int run_count = 0;

  for (size_t i = 0; i < n; ++i) {
    const uint16_t raw = raw_classes[i];

    if (raw == current_class) {
      // Matches accepted state — reset any pending transition
      run_count = 0;
      pending_class = current_class;
      filtered[i] = current_class;
    } else if (raw == pending_class) {
      // Continue building a run of the candidate class
      ++run_count;
      if (run_count >= hysteresis_window_) {
        // Transition confirmed — backfill the run to the new class
        current_class = pending_class;
        for (int j = static_cast<int>(i) - run_count + 1; j <= static_cast<int>(i); ++j) {
          filtered[j] = pending_class;
        }
        run_count = 0;
      } else {
        // Not yet confirmed — keep old class
        filtered[i] = current_class;
      }
    } else {
      // A different class appeared — reset pending to this new candidate
      pending_class = raw;
      run_count = 1;
      if (hysteresis_window_ <= 1) {
        current_class = raw;
        filtered[i] = raw;
        run_count = 0;
      } else {
        filtered[i] = current_class;
      }
    }
  }

  return filtered;
}

// ---------------------------------------------------------------------------
// Stage 3: group consecutive same-type poses into segments
// ---------------------------------------------------------------------------

std::vector<PathSplitter::Segment> PathSplitter::groupSegments(
  const std::vector<uint16_t> & classes)
{
  std::vector<Segment> segments;
  Segment current{classes[0], 0, 0};

  for (size_t i = 1; i < classes.size(); ++i) {
    if (classes[i] != current.class_type) {
      current.end_idx = i;
      segments.push_back(current);
      current = {classes[i], i, 0};
    }
  }
  // Close last segment
  current.end_idx = classes.size();
  segments.push_back(current);

  return segments;
}

// ---------------------------------------------------------------------------
// Stage 4: merge segments shorter than min_segment_poses into neighbors
// ---------------------------------------------------------------------------

void PathSplitter::mergeShortSegments(std::vector<Segment> & segments)
{
  if (segments.size() <= 1) {
    return;
  }

  bool changed = true;
  while (changed) {
    changed = false;
    for (size_t i = 0; i < segments.size(); ++i) {
      const size_t length = segments[i].end_idx - segments[i].start_idx;
      if (length < static_cast<size_t>(min_segment_poses_) && segments.size() > 1) {
        if (i > 0) {
          // Merge into left neighbor
          segments[i - 1].end_idx = segments[i].end_idx;
        } else {
          // First segment — merge into right neighbor
          segments[i + 1].start_idx = segments[i].start_idx;
        }
        segments.erase(segments.begin() + static_cast<ptrdiff_t>(i));
        changed = true;
        break;  // restart scan after mutation
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Stage 5: build ClassifiedPathArray with 1-pose overlap at boundaries
// ---------------------------------------------------------------------------

nav2_msgs::msg::ClassifiedPathArray PathSplitter::buildResult(
  const nav_msgs::msg::Path & path,
  const std::vector<Segment> & segments)
{
  nav2_msgs::msg::ClassifiedPathArray result;
  result.paths.reserve(segments.size());

  for (size_t s = 0; s < segments.size(); ++s) {
    nav2_msgs::msg::ClassifiedPath cp;
    cp.class_type = segments[s].class_type;
    cp.path.header = path.header;

    // Copy poses for this segment [start_idx, end_idx)
    for (size_t i = segments[s].start_idx; i < segments[s].end_idx; ++i) {
      cp.path.poses.push_back(path.poses[i]);
    }

    // Add 1-pose overlap: duplicate first pose of next segment as trailing pose
    if (s + 1 < segments.size()) {
      cp.path.poses.push_back(path.poses[segments[s + 1].start_idx]);
    }

    result.paths.push_back(cp);
  }

  return result;
}

}  // namespace nav2_planner
