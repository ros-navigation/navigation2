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

#ifndef NAV2_PLANNER__PATH_SPLITTER_HPP_
#define NAV2_PLANNER__PATH_SPLITTER_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_msgs/msg/classified_path.hpp"
#include "nav2_msgs/msg/classified_path_array.hpp"
#include "nav2_planner/pose_classifier.hpp"

namespace nav2_planner
{

/**
 * @class PathSplitter
 * @brief Splits a planner path into ClassifiedPathArray segments.
 *
 * Pipeline:
 *   1. Classify each pose via PoseClassifier (first-match plugin dispatch)
 *   2. Apply hysteresis filter (N consecutive flips required to change state)
 *   3. Group consecutive same-type poses into segments
 *   4. Merge segments shorter than min_segment_poses into neighbors
 *   5. Add 1-pose overlap at segment boundaries (controller handoff)
 *
 * Parameters (under path_splitter namespace):
 *   - hysteresis_window (int, default 3): consecutive pose count to confirm a transition
 *   - min_segment_poses (int, default 5): segments shorter than this merge into neighbors
 */
class PathSplitter
{
public:
  struct SplitResult
  {
    std::vector<nav2_msgs::msg::ClassifiedPose> classified_poses;
    nav2_msgs::msg::ClassifiedPathArray classified_path_array;
  };

  PathSplitter() = default;
  ~PathSplitter() = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);
  void cleanup();

  /**
   * @brief Split a path into classified segments.
   * @param path            Input path from the planner
   * @param pose_classifier Reference to the PoseClassifier for per-pose classification
   * @return SplitResult containing raw classified poses and the final ClassifiedPathArray
   */
  SplitResult splitPath(
    const nav_msgs::msg::Path & path,
    PoseClassifier & pose_classifier);

private:
  struct Segment
  {
    uint16_t class_type;
    size_t start_idx;  // inclusive index into raw_classes
    size_t end_idx;    // exclusive
  };

  std::vector<uint16_t> classifyAllPoses(
    const nav_msgs::msg::Path & path,
    PoseClassifier & pose_classifier);

  std::vector<uint16_t> applyHysteresis(
    const std::vector<uint16_t> & raw_classes);

  std::vector<Segment> groupSegments(
    const std::vector<uint16_t> & classes);

  void mergeShortSegments(std::vector<Segment> & segments);

  nav2_msgs::msg::ClassifiedPathArray buildResult(
    const nav_msgs::msg::Path & path,
    const std::vector<Segment> & segments);

  int hysteresis_window_{3};
  int min_segment_poses_{5};
  rclcpp::Logger logger_{rclcpp::get_logger("PathSplitter")};
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PATH_SPLITTER_HPP_
