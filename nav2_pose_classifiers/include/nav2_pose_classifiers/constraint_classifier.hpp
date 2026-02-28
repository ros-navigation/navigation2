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

#ifndef NAV2_POSE_CLASSIFIERS__CONSTRAINT_CLASSIFIER_HPP_
#define NAV2_POSE_CLASSIFIERS__CONSTRAINT_CLASSIFIER_HPP_

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav2_pose_classifiers/classifier_base.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/msg/path_classes.hpp"
#include "tf2/utils.h"
#include "clipper.hpp"

namespace nav2_pose_classifiers
{

/**
 * @class ConstraintClassifier
 * @brief Classifies a pose as CONSTRAINT_SPACE by iteratively inflating the
 *        robot footprint and detecting LETHAL cost on opposite edges.
 *
 * Algorithm:
 *   1. For each edge, find its geometric opposite by casting a ray from the
 *      edge midpoint through the polygon centroid — the edge it hits on the
 *      other side is the opposite. Stored as a 1:1 map.
 *   2. Starting from the base footprint, inflate by inflation_resolution each
 *      iteration using Clipper1 mitered offset (ClipperOffset + jtMiter).
 *   3. Loop limit: total inflation = max_constraint_clearance.
 *   4. Each iteration: compute lineCost per inflated edge.
 *      - If an edge hits LETHAL_OBSTACLE → record its index.
 *      - If the opposite edge of any recorded index also hits LETHAL → CONSTRAINT.
 *   5. Early exit on first opposite-pair LETHAL match.
 *
 * Parameters:
 *   - class_type (int):                 class enum value (default 1 = CONSTRAINT_SPACE)
 *   - inflation_resolution (double):    step size per iteration in metres (default 0.20)
 *   - max_constraint_clearance (double): max inflation distance from footprint edge (default 1.0)
 */
class ConstraintClassifier : public ClassifierBase
{
public:
  ConstraintClassifier() = default;
  ~ConstraintClassifier() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool matches(const geometry_msgs::msg::PoseStamped & pose) override;
  uint16_t classType() override;

private:
  /**
   * @brief Inflate footprint by delta metres using Clipper1 mitered offset.
   *        Converts double coords → Clipper integer coords (micrometre scale),
   *        runs ClipperOffset(jtMiter), converts back.
   */
  nav2_costmap_2d::Footprint inflateFootprint(
    const nav2_costmap_2d::Footprint & fp, double delta) const;

  /**
   * @brief Rotate + translate robot-frame footprint to world frame.
   *        Same transform as footprintCostAtPose.
   */
  nav2_costmap_2d::Footprint orientFootprint(
    const nav2_costmap_2d::Footprint & fp,
    double x, double y, double cos_th, double sin_th) const;

  /**
   * @brief Build a 1:1 map of opposite edges using centroid ray-casting.
   *        For each edge i, cast a ray from the edge midpoint through the
   *        polygon centroid. The edge the ray hits on the other side is the
   *        opposite of edge i.
   * @return Vector where opposite[i] = index of edge opposite to i.
   *         Set to n (invalid) if no opposite found (degenerate case).
   */
  std::vector<size_t> buildOppositePairs(
    const nav2_costmap_2d::Footprint & fp) const;

  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("ConstraintClassifier")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_;

  uint16_t class_type_;
  double inflation_resolution_;
  double max_constraint_clearance_;

  // Cached footprint + opposite-pairs (recomputed only when footprint changes)
  nav2_costmap_2d::Footprint raw_fp_;
  std::vector<size_t> opposites_;
};

}  // namespace nav2_pose_classifiers

#endif  // NAV2_POSE_CLASSIFIERS__CONSTRAINT_CLASSIFIER_HPP_
