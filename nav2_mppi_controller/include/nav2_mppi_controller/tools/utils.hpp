// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

// xtensor creates warnings that needs to be ignored as we are building with -Werror
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>
#pragma GCC diagnostic pop

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/goal_checker.hpp"

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

#define M_PIF 3.141592653589793238462643383279502884e+00F
#define M_PIF_2 1.5707963267948966e+00F

namespace mppi::utils
{
using xt::evaluation_strategy::immediate;

/**
 * @brief Convert data into pose
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @return Pose object
 */
inline geometry_msgs::msg::Pose createPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  return pose;
}

/**
 * @brief Convert data into scale
 * @param x X scale
 * @param y Y scale
 * @param z Z scale
 * @return Scale object
 */
inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

/**
 * @brief Convert data into color
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param a Alpha component (transparency)
 * @return Color object
 */
inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * @brief Convert data into a Maarker
 * @param id Marker ID
 * @param pose Marker pose
 * @param scale Marker scale
 * @param color Marker color
 * @param frame Reference frame to use
 * @return Visualization Marker
 */
inline visualization_msgs::msg::Marker createMarker(
  int id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const std::string & frame_id, const std::string & ns)
{
  using visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0, 0);
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;

  return twist;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param vy Y velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float vy, float wz, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  auto twist = toTwistStamped(vx, wz, stamp, frame);
  twist.twist.linear.y = vy;

  return twist;
}

/**
 * @brief Convert path to a tensor
 * @param path Path to convert
 * @return Path tensor
 */
inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return result;
}

/**
 * @brief Check if the robot pose is within the Goal Checker's tolerances to goal
 * @param global_checker Pointer to the goal checker
 * @param robot Pose of robot
 * @param goal Goal pose
 * @return bool If robot is within goal checker tolerances to the goal
 */
inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::Pose & robot,
  const geometry_msgs::msg::Pose & goal)
{
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal.position.x;
    auto dy = robot.position.y - goal.position.y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }
  }

  return false;
}

/**
 * @brief Check if the robot pose is within tolerance to the goal
 * @param pose_tolerance Pose tolerance to use
 * @param robot Pose of robot
 * @param goal Goal pose
 * @return bool If robot is within tolerance to the goal
 */
inline bool withinPositionGoalTolerance(
  float pose_tolerance,
  const geometry_msgs::msg::Pose & robot,
  const geometry_msgs::msg::Pose & goal)
{
  const double & dist_sq =
    std::pow(goal.position.x - robot.position.x, 2) +
    std::pow(goal.position.y - robot.position.y, 2);

  const float pose_tolerance_sq = pose_tolerance * pose_tolerance;

  if (dist_sq < pose_tolerance_sq) {
    return true;
  }

  return false;
}

/**
  * @brief normalize
  * Normalizes the angle to be -M_PIF circle to +M_PIF circle
  * It takes and returns radians.
  * @param angles Angles to normalize
  * @return normalized angles
  */
template<typename T>
auto normalize_angles(const T & angles)
{
  auto theta = xt::eval(xt::fmod(angles + M_PIF, 2.0f * M_PIF));
  return xt::eval(xt::where(theta < 0.0f, theta + M_PIF, theta - M_PIF));
}

/**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  * @param from Start angle
  * @param to End angle
  * @return Shortest distance between angles
  */
template<typename F, typename T>
auto shortest_angular_distance(
  const F & from,
  const T & to)
{
  return normalize_angles(to - from);
}

/**
 * @brief Evaluate furthest point idx of data.path which is
 * nearset to some trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of furthest path point reached by a set of trajectories
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());

  const auto dx = data.path.x - traj_x;
  const auto dy = data.path.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0, min_id_by_path = 0;
  float min_distance_by_path = std::numeric_limits<float>::max();

  for (size_t i = 0; i < dists.shape(0); i++) {
    min_id_by_path = 0;
    min_distance_by_path = std::numeric_limits<float>::max();
    for (size_t j = max_id_by_trajectories; j < dists.shape(1); j++) {
      const float cur_dist = dists(i, j);
      if (cur_dist < min_distance_by_path) {
        min_distance_by_path = cur_dist;
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  return max_id_by_trajectories;
}

/**
 * @brief evaluate path furthest point if it is not set
 * @param data Data to use
 */
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief evaluate path costs
 * @param data Data to use
 */
inline void findPathCosts(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto * costmap = costmap_ros->getCostmap();
  unsigned int map_x, map_y;
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  data.path_pts_valid = std::vector<bool>(path_segments_count, false);
  const bool tracking_unknown = costmap_ros->getLayeredCostmap()->isTrackingUnknown();
  for (unsigned int idx = 0; idx < path_segments_count; idx++) {
    if (!costmap->worldToMap(data.path.x(idx), data.path.y(idx), map_x, map_y)) {
      (*data.path_pts_valid)[idx] = false;
      continue;
    }

    switch (costmap->getCost(map_x, map_y)) {
      case (nav2_costmap_2d::LETHAL_OBSTACLE):
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (nav2_costmap_2d::NO_INFORMATION):
        (*data.path_pts_valid)[idx] = tracking_unknown ? true : false;
        continue;
    }

    (*data.path_pts_valid)[idx] = true;
  }
}

/**
 * @brief evaluate path costs if it is not set
 * @param data Data to use
 */
inline void setPathCostsIfNotSet(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (!data.path_pts_valid) {
    findPathCosts(data, costmap_ros);
  }
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param forward_preference If reversing direction is valid
 * @return Angle between two points
 */
inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose, double point_x, double point_y, bool forward_preference)
{
  float pose_x = pose.position.x;
  float pose_y = pose.position.y;
  float pose_yaw = tf2::getYaw(pose.orientation);

  float yaw = atan2f(point_y - pose_y, point_x - pose_x);

  // If no preference for forward, return smallest angle either in heading or 180 of heading
  if (!forward_preference) {
    return std::min(
      fabs(angles::shortest_angular_distance(yaw, pose_yaw)),
      fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PIF))));
  }

  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param point_yaw Yaw of the point to consider along Z axis
 * @return Angle between two points
 */
inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose,
  double point_x, double point_y, double point_yaw)
{
  float pose_x = static_cast<float>(pose.position.x);
  float pose_y = static_cast<float>(pose.position.y);
  float pose_yaw = static_cast<float>(tf2::getYaw(pose.orientation));

  float yaw = atan2f(static_cast<float>(point_y) - pose_y, static_cast<float>(point_x) - pose_x);

  if (fabs(angles::shortest_angular_distance(yaw, static_cast<float>(point_yaw))) > M_PIF_2) {
    yaw = angles::normalize_angle(yaw + M_PIF);
  }

  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Apply Savisky-Golay filter to optimal trajectory
 * @param control_sequence Sequence to apply filter to
 * @param control_history Recent set of controls for edge-case handling
 * @param Settings Settings to use
 */
inline void savitskyGolayFilter(
  models::ControlSequence & control_sequence,
  std::array<mppi::models::Control, 4> & control_history,
  const models::OptimizerSettings & settings)
{
  // Savitzky-Golay Quadratic, 9-point Coefficients
  xt::xarray<float> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
  filter /= 231.0;

  // Too short to smooth meaningfully
  const unsigned int num_sequences = control_sequence.vx.shape(0) - 1;
  if (num_sequences < 20) {
    return;
  }

  auto applyFilter = [&](const xt::xarray<float> & data) -> float {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<float, 1> & sequence, const xt::xtensor<float, 1> & initial_sequence,
    const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
      float pt_m4 = hist_0;
      float pt_m3 = hist_1;
      float pt_m2 = hist_2;
      float pt_m1 = hist_3;
      float pt = initial_sequence(0);
      float pt_p1 = initial_sequence(1);
      float pt_p2 = initial_sequence(2);
      float pt_p3 = initial_sequence(3);
      float pt_p4 = initial_sequence(4);

      for (unsigned int idx = 0; idx != num_sequences; idx++) {
        sequence(idx) = applyFilter({pt_m4, pt_m3, pt_m2, pt_m1, pt, pt_p1, pt_p2, pt_p3, pt_p4});
        pt_m4 = pt_m3;
        pt_m3 = pt_m2;
        pt_m2 = pt_m1;
        pt_m1 = pt;
        pt = pt_p1;
        pt_p1 = pt_p2;
        pt_p2 = pt_p3;
        pt_p3 = pt_p4;

        if (idx + 5 < num_sequences) {
          pt_p4 = initial_sequence(idx + 5);
        } else {
          // Return the last point
          pt_p4 = initial_sequence(num_sequences);
        }
      }
    };

  // Filter trajectories
  const models::ControlSequence initial_control_sequence = control_sequence;
  applyFilterOverAxis(
    control_sequence.vx, initial_control_sequence.vx, control_history[0].vx,
    control_history[1].vx, control_history[2].vx, control_history[3].vx);
  applyFilterOverAxis(
    control_sequence.vy, initial_control_sequence.vy, control_history[0].vy,
    control_history[1].vy, control_history[2].vy, control_history[3].vy);
  applyFilterOverAxis(
    control_sequence.wz, initial_control_sequence.wz, control_history[0].wz,
    control_history[1].wz, control_history[2].wz, control_history[3].wz);

  // Update control history
  unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  control_history[0] = control_history[1];
  control_history[1] = control_history[2];
  control_history[2] = control_history[3];
  control_history[3] = {
    control_sequence.vx(offset),
    control_sequence.vy(offset),
    control_sequence.wz(offset)};
}

/**
 * @brief Find the iterator of the first pose at which there is an inversion on the path,
 * @param path to check for inversion
 * @return the first point after the inversion found in the path
 */
inline unsigned int findFirstPathInversion(nav_msgs::msg::Path & path)
{
  // At least 3 poses for a possible inversion
  if (path.poses.size() < 3) {
    return path.poses.size();
  }

  // Iterating through the path to determine the position of the path inversion
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    float oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    float oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    float ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existance of cusp, in the path, using the dot product.
    float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0f) {
      return idx + 1;
    }
  }

  return path.poses.size();
}

/**
 * @brief Find and remove poses after the first inversion in the path
 * @param path to check for inversion
 * @return The location of the inversion, return 0 if none exist
 */
inline unsigned int removePosesAfterFirstInversion(nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path cropped_path = path;
  const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
  if (first_after_inversion == path.poses.size()) {
    return 0u;
  }

  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
  path = cropped_path;
  return first_after_inversion;
}

/**
 * @brief Compare to trajectory points to find closest path point along integrated distances
 * @param vec Vect to check
 * @return dist Distance to look for
 * @return init Starting index to indec from
 */
inline unsigned int findClosestPathPt(
  const std::vector<float> & vec, const float dist, const unsigned int init = 0u)
{
  float distim1 = init != 0u ? vec[init] : 0.0f;  // First is 0, no accumulated distance yet
  float disti = 0.0f;
  const unsigned int size = vec.size();
  for (unsigned int i = init + 1; i != size; i++) {
    disti = vec[i];
    if (disti > dist) {
      if (i > 0 && dist - distim1 < disti - dist) {
        return i - 1;
      }
      return i;
    }
    distim1 = disti;
  }
  return size - 1;
}

// A struct to hold pose data in floating point resolution
struct Pose2D
{
  float x, y, theta;
};

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
