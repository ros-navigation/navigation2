// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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
    tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "curvature_lookahead_point", 1);
  is_rotating_to_heading_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "is_rotating_to_heading", 1);
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
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = getLookAheadPoint(
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
    angular_vel = linear_vel * regulation_curvature;
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

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan,
  bool interpolate_after_goal)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    if (interpolate_after_goal) {
      auto last_pose_it = std::prev(transformed_plan.poses.end());
      auto prev_last_pose_it = std::prev(last_pose_it);

      double end_path_orientation = atan2(
        last_pose_it->pose.position.y - prev_last_pose_it->pose.position.y,
        last_pose_it->pose.position.x - prev_last_pose_it->pose.position.x);

      // Project the last segment out to guarantee it is beyond the look ahead
      // distance
      auto projected_position = last_pose_it->pose.position;
      projected_position.x += cos(end_path_orientation) * lookahead_dist;
      projected_position.y += sin(end_path_orientation) * lookahead_dist;

      // Use the circle intersection to find the position at the correct look
      // ahead distance
      const auto interpolated_position = circleSegmentIntersection(
        last_pose_it->pose.position, projected_position, lookahead_dist);

      geometry_msgs::msg::PoseStamped interpolated_pose;
      interpolated_pose.header = last_pose_it->header;
      interpolated_pose.pose.position = interpolated_position;
      return interpolated_pose;
    } else {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
  } else if (goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
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

    /* Checking for the existance of cusp, in the path, using the dot product
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
