// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2025 Fumiya Ohnishi
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
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "angles/angles.h"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/controller_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_, costmap_->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(
    params_->transform_tolerance, tf_, costmap_ros_);

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan");
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point");
  curvature_carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "curvature_lookahead_point");
  is_rotating_to_heading_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "is_rotating_to_heading");
}

void RegulatedPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  carrot_pub_.reset();
  curvature_carrot_pub_.reset();
  is_rotating_to_heading_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_carrot_pub_->on_activate();
  is_rotating_to_heading_pub_->on_activate();
}

void RegulatedPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_carrot_pub_->on_deactivate();
  is_rotating_to_heading_pub_->on_deactivate();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = params_->lookahead_dist;
  if (params_->use_velocity_scaled_lookahead_dist) {
    lookahead_dist = fabs(speed.linear.x) * params_->lookahead_time;
    lookahead_dist = std::clamp(
      lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
  }

  return lookahead_dist;
}

double calculateCurvature(geometry_msgs::msg::Point lookahead_point)
{
  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (lookahead_point.x * lookahead_point.x) +
    (lookahead_point.y * lookahead_point.y);

  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

void RegulatedPurePursuitController::computeOptimalVelocityUsingDynamicWindow(
    const double curvature,
    const geometry_msgs::msg::Twist current_speed,
    const double regulated_linear_vel,
    double & optimal_linear_vel,
    double & optimal_angular_vel
  )
{
  // ---- Parameters (assumed available in this scope) ----
  const double A_MAX  = params_->max_linear_accel;      // A_MAX
  const double V_MAX  = params_->desired_linear_vel;    // V_MAX
  const double AW_MAX = params_->max_angular_accel;     // AW_MAX
  const double W_MAX  = params_->desired_angular_vel;   // W_MAX
  const double DT     = control_duration_;     // DT

  const double V_MIN = -V_MAX;
  const double W_MIN = -W_MAX;

  // ---- 1) Dynamic Window ----
  double dw_vmax = std::min(current_speed.linear.x  + A_MAX  * DT, V_MAX);
  const double dw_vmin = std::max(current_speed.linear.x  - A_MAX  * DT, V_MIN);
  double dw_wmax = std::min(current_speed.angular.z + AW_MAX * DT, W_MAX);
  const double dw_wmin = std::max(current_speed.angular.z - AW_MAX * DT, W_MIN);

  // Reflect regulated v (tighten upper limit)
  if (dw_vmax > regulated_linear_vel) {
    dw_vmax = std::max(dw_vmin, regulated_linear_vel);
  }

  const double k = curvature;

  // ---- Curvature is 0 (w = 0) ----
  if (k == 0.0) {
    // If w=0 is within DW, then the maximum linear speed is adopted as it is.
    if (dw_wmin <= 0.0 && 0.0 <= dw_wmax) {
      optimal_linear_vel  = dw_vmax; // Always maximum v
      optimal_angular_vel = 0.0;
      return;
    }
    // If w=0 is outside, choose the side closer to w=0 and with smaller |w|.
    const double w_choice = (std::abs(dw_wmin) <= std::abs(dw_wmax)) ? dw_wmin : dw_wmax;
    optimal_linear_vel  = dw_vmax;   // Always maximum v
    optimal_angular_vel = w_choice;
    return;
  }

  // ---- 2) Select 'max v' from the candidates in the DW among the intersections. ----
  double best_v = -1e300;     // Initial value for maximization
  double best_w = 0.0;

  // Intersection with vertical edges
  {
    const double v1 = dw_vmin;
    const double w1 = k * v1;
    if (w1 >= dw_wmin && w1 <= dw_wmax) {
      if (v1 > best_v) { best_v = v1; best_w = w1; }
    }
  }
  {
    const double v2 = dw_vmax;
    const double w2 = k * v2;
    if (w2 >= dw_wmin && w2 <= dw_wmax) {
      if (v2 > best_v) { best_v = v2; best_w = w2; }
    }
  }

  // Intersection with horizontal edge (k ! = 0)
  {
    const double v3 = dw_wmin / k;
    if (v3 >= dw_vmin && v3 <= dw_vmax) {
      const double w3 = dw_wmin;
      if (v3 > best_v) { best_v = v3; best_w = w3; }
    }
  }
  {
    const double v4 = dw_wmax / k;
    if (v4 >= dw_vmin && v4 <= dw_vmax) {
      const double w4 = dw_wmax;
      if (v4 > best_v) { best_v = v4; best_w = w4; }
    }
  }

  if (best_v > -1e290) {
    // Intersection found → Adopt max. v
    optimal_linear_vel  = best_v;
    optimal_angular_vel = best_w;
    return;
  }

  // ---- 3) If no intersection exists: Select the one with the smallest Euclidean distance to the line w = k v among the 4 corners
  struct Corner { double v; double w; };
  const Corner corners[4] = {
    {dw_vmin, dw_wmin},
    {dw_vmin, dw_wmax},
    {dw_vmax, dw_wmin},
    {dw_vmax, dw_wmax}
  };

  const double denom = std::sqrt(k * k + 1.0); // Just sqrt once

  auto euclid_dist = [&](const Corner &c) -> double {
    // Distance from point (v, w) to line w - k v = 0
    return std::abs(k * c.v - c.w) / denom;
  };

  double best_dist = 1e300;
  best_v = corners[0].v;
  best_w = corners[0].w;

  for (int i = 0; i < 4; ++i) {
    const double d = euclid_dist(corners[i]);
    // 1) Smaller distance → Adopted
    // 2) If distances are equal (~1e-12) → Choose the one with larger v (acceleration policy)
    if (d < best_dist || (std::abs(d - best_dist) <= 1e-12 && corners[i].v > best_v)) {
      best_dist = d;
      best_v = corners[i].v;
      best_w = corners[i].w;
    }
  }

  optimal_linear_vel  = best_v;
  optimal_angular_vel = best_w;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);
  global_path_pub_->publish(transformed_plan);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);
  double curv_lookahead_dist = params_->curvature_lookahead_dist;

  // Check for reverse driving
  if (params_->allow_reversing) {
    // Cusp check
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
    if (dist_to_cusp < curv_lookahead_dist) {
      curv_lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = nav2_util::getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = nav2_util::getLookAheadPoint(
      curv_lookahead_dist,
      transformed_plan, params_->interpolate_curvature_after_goal);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    curvature_carrot_pub_->publish(createCarrotMsg(curvature_lookahead_pose));
  }

  // Setting the velocity direction
  double x_vel_sign = 1.0;
  if (params_->allow_reversing) {
    x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = params_->desired_linear_vel;

  // Make sure we're in compliance with basic constraints
  // For shouldRotateToPath, using x_vel_sign in order to support allow_reversing
  // and rotate_to_path_carrot_pose for the direction carrot pose:
  //        - equal to "normal" carrot_pose when curvature_lookahead_pose = false
  //        - otherwise equal to curvature_lookahead_pose (which can be interpolated after goal)
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    is_rotating_to_heading_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign)) {
    is_rotating_to_heading_ = true;
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    is_rotating_to_heading_ = false;
    applyConstraints(
      regulation_curvature, speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, x_vel_sign);

    if (cancelling_) {
      const double & dt = control_duration_;
      linear_vel = speed.linear.x - x_vel_sign * dt * params_->cancel_deceleration;

      if (x_vel_sign > 0) {
        if (linear_vel <= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      } else {
        if (linear_vel >= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      }
    }

    // Apply curvature to angular velocity after constraining linear velocity
    if (params_->use_dynamic_window == false){
      angular_vel = linear_vel * regulation_curvature;
    }
    else{
      // compute optimal path tracking velocity commands considering velocity and acceleration constraints
      const double regulated_linear_vel = linear_vel;
      if (params_->velocity_feedback == "CLOSED_LOOP"){
        // using odom velocity as a current velocity (not recommended)
        computeOptimalVelocityUsingDynamicWindow(regulation_curvature, speed, regulated_linear_vel, linear_vel, angular_vel);
      }
      else{
        // using last command velocity as a current velocity (recommended)
        computeOptimalVelocityUsingDynamicWindow(regulation_curvature, last_command_velocity_, regulated_linear_vel, linear_vel, angular_vel);
      }
    }
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
  {
    throw nav2_core::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
  }

  // Publish whether we are rotating to goal heading
  std_msgs::msg::Bool is_rotating_to_heading_msg;
  is_rotating_to_heading_msg.data = is_rotating_to_heading_;
  is_rotating_to_heading_pub_->publish(is_rotating_to_heading_msg);

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  
  last_command_velocity_ = cmd_vel.twist;

  return cmd_vel;
}

bool RegulatedPurePursuitController::cancel()
{
  // if false then publish zero velocity
  if (!params_->use_cancel_deceleration) {
    return true;
  }
  cancelling_ = true;
  return finished_cancelling_;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path,
  double & x_vel_sign)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  // In case we are reversing
  if (x_vel_sign < 0.0) {
    angle_to_path = angles::normalize_angle(angle_to_path + M_PI);
  }
  return params_->use_rotate_to_heading &&
         fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  if (!params_->use_rotate_to_heading) {
    return false;
  }

  double dist_to_goal = std::hypot(
    carrot_pose.pose.position.x, carrot_pose.pose.position.y);

  if (params_->stateful) {
    if (!has_reached_xy_tolerance_ && dist_to_goal < goal_dist_tol_) {
      has_reached_xy_tolerance_ = true;
    }
    return has_reached_xy_tolerance_;
  }

  return dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * params_->rotate_to_heading_angular_vel;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop = std::sqrt(2 * params_->max_angular_accel * fabs(angle_to_path));
  if (fabs(angular_vel) > max_vel_to_stop) {
    angular_vel = sign * max_vel_to_stop;
  }
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel, cost_vel = linear_vel;

  // limit the linear velocity by curvature
  if (params_->use_regulated_linear_velocity_scaling) {
    curvature_vel = heuristics::curvatureConstraint(
      linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
  }

  // limit the linear velocity by proximity to obstacles
  if (params_->use_cost_regulated_linear_velocity_scaling) {
    cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_ros_, params_);
  }

  // Use the lowest of the 2 constraints, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);

  // Apply constraint to reduce speed on approach to the final goal pose
  linear_vel = heuristics::approachVelocityConstraint(
    linear_vel, path, params_->min_approach_linear_velocity,
    params_->approach_velocity_scaling_dist);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  has_reached_xy_tolerance_ = false;
  path_handler_->setPlan(path);
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    params_->desired_linear_vel = params_->base_desired_linear_vel;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      params_->desired_linear_vel = params_->base_desired_linear_vel * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      params_->desired_linear_vel = speed_limit;
    }
  }
}

void RegulatedPurePursuitController::reset()
{
  cancelling_ = false;
  finished_cancelling_ = false;
  has_reached_xy_tolerance_ = false;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existence of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    const double dot_prod = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_prod < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }

    if (
      (hypot(oa_x, oa_y) == 0.0 &&
      transformed_plan.poses[pose_id - 1].pose.orientation !=
      transformed_plan.poses[pose_id].pose.orientation)
      ||
      (hypot(ab_x, ab_y) == 0.0 &&
      transformed_plan.poses[pose_id].pose.orientation !=
      transformed_plan.poses[pose_id + 1].pose.orientation))
    {
      // returning the distance since the points overlap
      // but are not simply duplicate points (e.g. in place rotation)
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}
}  // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav2_core::Controller)
