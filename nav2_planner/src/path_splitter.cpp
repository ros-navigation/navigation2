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
  PoseClassifier & pose_classifier,
  bool build_classified_poses)
{
  SplitResult result;

  if (path.poses.empty()) {
    return result;
  }

  // Stage 1+2+3 (single pass): classify each pose, apply hysteresis, group into segments.
  // Also builds raw classified_poses for visualization if requested.
  auto segments = classifyAndGroup(
    path, pose_classifier,
    build_classified_poses ? &result.classified_poses : nullptr);

  // Stage 4: merge segments shorter than min_segment_poses into neighbors
  mergeShortSegments(segments);

  // Stage 5: build ClassifiedPathArray with 1-pose overlap at boundaries
  result.classified_path_array = buildResult(path, segments);

  RCLCPP_DEBUG(
    logger_, "PathSplitter: %zu poses -> %zu segments",
    path.poses.size(), result.classified_path_array.paths.size());

  return result;
}

// ---------------------------------------------------------------------------
// Stage 1+2+3 (single pass): classify, hysteresis filter, group into segments
//
// For each pose:
//   - Stage 1: call pose_classifier.classify() to get the raw class
//   - Stage 2: apply hysteresis — only confirm a class transition after
//              hysteresis_window_ consecutive poses of the new class.
//              On confirmation, backfill by adjusting the segment boundary
//              to the start of the confirmed run.
//   - Stage 3: track segment boundaries inline — a new segment starts
//              whenever the filtered (post-hysteresis) class changes.
//
// Optionally builds classified_poses (raw, pre-hysteresis) for visualization.
// ---------------------------------------------------------------------------

std::vector<PathSplitter::Segment> PathSplitter::classifyAndGroup(
  const nav_msgs::msg::Path & path,
  PoseClassifier & pose_classifier,
  std::vector<nav2_msgs::msg::ClassifiedPose> * classified_poses)
{
  const size_t n = path.poses.size();
  std::vector<Segment> segments;

  if (classified_poses) {
    classified_poses->reserve(n);
  }

  // --- Initialize with first pose ---
  uint16_t first_raw = pose_classifier.classify(path.poses[0]);

  // Hysteresis state
  uint16_t current_class = first_raw;   // accepted (post-hysteresis) class
  uint16_t pending_class = current_class;
  int run_count = 0;

  // Segment tracking
  Segment current_segment{current_class, 0, 0};

  // Build raw classified pose for first pose
  if (classified_poses) {
    nav2_msgs::msg::ClassifiedPose cp;
    cp.pose = path.poses[0];
    cp.class_type = first_raw;
    classified_poses->push_back(cp);
  }

  // --- Single pass over remaining poses ---
  for (size_t i = 1; i < n; ++i) {
    // Stage 1: classify this pose
    uint16_t raw = pose_classifier.classify(path.poses[i]);

    // Build raw classified pose for visualization (pre-hysteresis)
    if (classified_poses) {
      nav2_msgs::msg::ClassifiedPose cp;
      cp.pose = path.poses[i];
      cp.class_type = raw;
      classified_poses->push_back(cp);
    }

    // Stage 2+3: hysteresis filter + inline segment grouping
    if (raw == current_class) {
      // Matches accepted state — reset any pending transition.
      // Pose stays in current_segment, nothing to do.
      run_count = 0;
      pending_class = current_class;

    } else if (raw == pending_class) {
      // Continue building a run of the candidate class
      ++run_count;

      if (run_count >= hysteresis_window_) {
        // Transition confirmed — backfill by adjusting segment boundary.
        // The last run_count poses were tentatively in current_segment
        // but actually belong to the new class.
        size_t backfill_start = i - run_count + 1;

        // Close current segment at the backfill point
        current_segment.end_idx = backfill_start;
        if (current_segment.end_idx > current_segment.start_idx) {
          segments.push_back(current_segment);
        }

        // Start new segment from the backfill point with the confirmed class
        current_class = pending_class;
        current_segment = {current_class, backfill_start, 0};
        run_count = 0;
      }
      // If not yet confirmed, pose logically stays in current_segment

    } else {
      // A different class appeared — reset pending to this new candidate
      pending_class = raw;
      run_count = 1;

      if (hysteresis_window_ <= 1) {
        // Immediate transition (no hysteresis needed)
        current_segment.end_idx = i;
        if (current_segment.end_idx > current_segment.start_idx) {
          segments.push_back(current_segment);
        }
        current_class = raw;
        current_segment = {current_class, i, 0};
        run_count = 0;
      }
      // Otherwise, pose logically stays in current_segment
    }
  }

  // Close the last segment
  current_segment.end_idx = n;
  segments.push_back(current_segment);

  return segments;
}

// ---------------------------------------------------------------------------
// Stage 4: merge segments shorter than min_segment_poses into neighbors
//
// Short segments are absorbed into the left neighbor (preferred) or right
// neighbor (for the first segment). Loops until no short segments remain.
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
//
// Each segment becomes a ClassifiedPath with poses copied from the original
// path. The last pose of each segment (except the final one) is duplicated
// as the first pose of the next segment for smooth controller handoff.
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
