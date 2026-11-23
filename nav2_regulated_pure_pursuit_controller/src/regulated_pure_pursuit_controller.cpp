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
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/controller_utils.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, nav2::TransformBuffer::SharedPtr tf,
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
  clock_ = node->get_clock();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_, costmap_->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

  // Predicts the robot state when commands take effect, for latency compensation
  latency_compensator_ = std::make_unique<LatencyCompensator>(
    params_->linear_model, params_->angular_model);

  double control_frequency = 20.0;

  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

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
  carrot_pub_->on_activate();
  curvature_carrot_pub_->on_activate();
  is_rotating_to_heading_pub_->on_activate();
  param_handler_->activate();
}

void RegulatedPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  carrot_pub_->on_deactivate();
  curvature_carrot_pub_->on_deactivate();
  is_rotating_to_heading_pub_->on_deactivate();
  param_handler_->deactivate();
  last_command_velocity_ = geometry_msgs::msg::Twist();
  last_desired_velocity_ = geometry_msgs::msg::Twist();
  latency_compensator_->reset();
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
  nav2_core::GoalChecker * goal_checker,
  const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Transform the plan from costmap's global frame to robot base frame
  nav_msgs::msg::Path transformed_plan;
  if (!nav2_util::transformPathInTargetFrame(
      transformed_global_plan, transformed_plan, *tf_,
      costmap_ros_->getBaseFrameID(), costmap_ros_->getTransformTolerance()))
  {
    throw nav2_core::ControllerTFError(
      "Unable to transform plan pose into local frame");
  }

  // With latency compensation, track the path from the state the robot is predicted to be in
  // when this command takes effect, rather than the measured state (Smith predictor).
  const bool compensate_latency = params_->use_latency_compensation;
  // Commands are recorded when issued, and the prediction runs from the pose's measurement time,
  // so that commands issued since a stale pose are accounted for
  const rclcpp::Time now = clock_->now();
  rclcpp::Time measurement_time(pose.header.stamp, now.get_clock_type());
  const double pose_age = (now - measurement_time).seconds();
  if (pose_age < 0.0 || pose_age > costmap_ros_->getTransformTolerance()) {
    // Clock skew or mismatched clocks, so treat the pose as current
    if (compensate_latency) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "Robot pose is %.3fs old, outside of [0, transform_tolerance]. "
        "Treating it as current for latency compensation. Are the clocks synchronized?",
        pose_age);
    }
    measurement_time = now;
  }
  geometry_msgs::msg::PoseStamped tracking_pose = pose;
  geometry_msgs::msg::Twist tracking_speed = speed;
  PredictedState prediction;
  if (compensate_latency) {
    prediction = latency_compensator_->predict(speed, measurement_time, now);
    transformed_plan = transformPlanToPredictedFrame(transformed_plan, prediction);

    const double yaw = tf2::getYaw(pose.pose.orientation);
    tracking_pose.pose.position.x +=
      prediction.x * cos(yaw) - prediction.y * sin(yaw);
    tracking_pose.pose.position.y +=
      prediction.x * sin(yaw) + prediction.y * cos(yaw);
    tracking_pose.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw + prediction.yaw);
    tracking_speed.linear.x = prediction.linear_vel;
    tracking_speed.angular.z = prediction.angular_vel;
  }

  // Express a pose in the tracking frame in the robot base frame for visualization
  auto toBaseFrame = [&](geometry_msgs::msg::PoseStamped p) {
      if (compensate_latency) {
        const double x = p.pose.position.x, y = p.pose.position.y;
        p.pose.position.x = prediction.x + x * cos(prediction.yaw) - y * sin(prediction.yaw);
        p.pose.position.y = prediction.y + x * sin(prediction.yaw) + y * cos(prediction.yaw);
      }
      return p;
    };

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(tracking_speed);
  double curv_lookahead_dist = params_->curvature_lookahead_dist;

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = nav2_util::getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  carrot_pub_->publish(createCarrotMsg(toBaseFrame(carrot_pose)));

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = nav2_util::getLookAheadPoint(
      curv_lookahead_dist,
      transformed_plan, params_->interpolate_curvature_after_goal);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    curvature_carrot_pub_->publish(createCarrotMsg(toBaseFrame(curvature_lookahead_pose)));
  }

  // Setting the velocity direction
  double x_vel_sign = 1.0;
  if (params_->allow_reversing) {
    x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = params_->max_linear_vel;

  // When compensating latency, accelerate rotations from the last desired velocity, as the
  // measured velocity lags behind it, and begin slowing before the lag would cause overshoot
  geometry_msgs::msg::Twist rotate_base_speed = tracking_speed;
  double rotate_time_constant = 0.0;
  if (compensate_latency) {
    rotate_base_speed.angular.z = last_desired_velocity_.angular.z;
    rotate_time_constant = params_->angular_model.time_constant;
  }

  // Make sure we're in compliance with basic constraints
  // For shouldRotateToPath, using x_vel_sign in order to support allow_reversing
  // and rotate_to_path_carrot_pose for the direction carrot pose:
  //        - equal to "normal" carrot_pose when curvature_lookahead_pose = false
  //        - otherwise equal to curvature_lookahead_pose (which can be interpolated after goal)
  double angle_to_heading;
  // Feed the goal checker the GLOBAL-frame plan (same frame as `pose` / `global_goal`), not the
  // base_link-frame `transformed_plan` used for lookahead. The custom AxisGoalChecker derives the
  // path-progress direction from the plan geometry, so a base-frame plan combined with global
  // query/goal poses yields a wrong end-of-path direction and a spurious goal-reached. This keeps
  // the check identical to controller_server::isGoalReached(), which uses the global-frame plan.
  if (shouldRotateToGoalHeading(goal_checker, pose, global_goal, speed, transformed_global_plan)) {
    is_rotating_to_heading_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(
      linear_vel, angular_vel, angle_to_goal, rotate_base_speed, rotate_time_constant,
      tracking_speed.angular.z);
  } else if (shouldRotateToPath(rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign)) {
    is_rotating_to_heading_ = true;
    rotateToHeading(
      linear_vel, angular_vel, angle_to_heading, rotate_base_speed, rotate_time_constant,
      tracking_speed.angular.z);
  } else {
    is_rotating_to_heading_ = false;
    applyConstraints(
      regulation_curvature, tracking_speed,
      collision_checker_->costAtPose(
        tracking_pose.pose.position.x, tracking_pose.pose.position.y),
      transformed_plan, linear_vel, x_vel_sign);

    if (cancelling_) {
      const double & dt = control_duration_;
      linear_vel = tracking_speed.linear.x - x_vel_sign * dt * params_->cancel_deceleration;

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
    if (!params_->use_dynamic_window) {
      angular_vel = linear_vel * regulation_curvature;
    } else {
      // compute optimal path tracking velocity commands
      // considering velocity and acceleration constraints (DWPP)
      const double regulated_linear_vel = linear_vel;
      // using last command velocity as a current velocity
      const geometry_msgs::msg::Twist current_speed =
        compensate_latency ? last_desired_velocity_ : last_command_velocity_;

      std::tie(linear_vel, angular_vel) =
        dynamic_window_pure_pursuit::computeDynamicWindowVelocities(
        current_speed,
        params_->max_linear_vel,
        params_->min_linear_vel,
        params_->max_angular_vel,
        params_->min_angular_vel,
        params_->max_linear_accel,
        params_->max_linear_decel,
        params_->max_angular_accel,
        params_->max_angular_decel,
        regulated_linear_vel,
        regulation_curvature,
        x_vel_sign,
        control_duration_);
    }
  }

  // Collision checking on this velocity heading
  const double dist_to_path_end =
    nav2_util::geometry_utils::calculate_path_length(transformed_plan);
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(tracking_pose, linear_vel, angular_vel, carrot_dist,
    dist_to_path_end))
  {
    latency_compensator_->recordCommand(geometry_msgs::msg::Twist(), now);
    throw nav2_core::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
  }

  // Publish whether we are rotating to goal heading
  auto is_rotating_to_heading_msg = std::make_unique<std_msgs::msg::Bool>();
  is_rotating_to_heading_msg->data = is_rotating_to_heading_;
  is_rotating_to_heading_pub_->publish(std::move(is_rotating_to_heading_msg));

  // Shape the desired velocity into the command which achieves it on the modeled robot
  double cmd_linear_vel = linear_vel;
  double cmd_angular_vel = angular_vel;
  if (compensate_latency && (params_->use_lead_compensation || params_->invert_actuation)) {
    // Reaching the desired velocity within a single control period demands large, chattering
    // commands, so the lead aims to reach it over the lag's time constant. The lead never
    // reverses the linear direction, but may brake rotation against the desired direction.
    const auto & lin_model = params_->linear_model;
    const auto & ang_model = params_->angular_model;
    cmd_linear_vel = LatencyCompensator::shapeCommand(
      linear_vel, prediction.linear_vel, lin_model,
      std::max(control_duration_, lin_model.time_constant),
      params_->use_lead_compensation, params_->invert_actuation, false);
    cmd_angular_vel = LatencyCompensator::shapeCommand(
      angular_vel, prediction.angular_vel, ang_model,
      std::max(control_duration_, ang_model.time_constant),
      params_->use_lead_compensation, params_->invert_actuation, true);

    if (params_->invert_actuation) {
      // Path tracking feedback raises an angular correction rounded to zero, but nothing raises
      // a linear velocity rounded to zero, so modulate it to average to the desired velocity
      cmd_linear_vel = latency_compensator_->modulateDeadband(cmd_linear_vel, lin_model.deadband);
      cmd_angular_vel =
        LatencyCompensator::roundToDeadband(cmd_angular_vel, ang_model.deadband);
    }

    // Bound the shaped command by the platform limits, scaled by the actuation gain
    const double lin_gain = params_->invert_actuation ? lin_model.gain : 1.0;
    const double ang_gain = params_->invert_actuation ? ang_model.gain : 1.0;
    const double lin_limit = std::max(fabs(linear_vel), params_->max_linear_vel) / lin_gain;
    const double ang_limit = std::max(fabs(angular_vel), params_->max_angular_vel) / ang_gain;
    cmd_linear_vel = std::clamp(cmd_linear_vel, -lin_limit, lin_limit);
    cmd_angular_vel = std::clamp(cmd_angular_vel, -ang_limit, ang_limit);
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = cmd_linear_vel;
  cmd_vel.twist.angular.z = cmd_angular_vel;

  // For dynamic window scaling in open-loop speed control
  last_command_velocity_ = cmd_vel.twist;
  last_desired_velocity_.linear.x = linear_vel;
  last_desired_velocity_.angular.z = angular_vel;
  latency_compensator_->recordCommand(cmd_vel.twist, now);

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
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const geometry_msgs::msg::Twist & speed,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Whether we should rotate robot to goal heading
  if (!params_->use_rotate_to_heading) {
    return false;
  }
  return goal_checker->isGoalXYReached(robot_pose.pose, goal_pose.pose, speed,
    transformed_plan);
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed,
  const double & time_constant, const double & robot_angular_vel)
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
  double max_vel_to_stop = LatencyCompensator::rotateStoppingSpeed(
    angle_to_path, params_->max_angular_accel, time_constant, robot_angular_vel);
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
  linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->max_linear_vel);
  linear_vel = sign * linear_vel;
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformPlanToPredictedFrame(
  const nav_msgs::msg::Path & path, const PredictedState & prediction)
{
  nav_msgs::msg::Path predicted_path;
  predicted_path.header = path.header;
  predicted_path.poses.reserve(path.poses.size());

  const double c = cos(prediction.yaw);
  const double s = sin(prediction.yaw);
  for (const auto & pose : path.poses) {
    geometry_msgs::msg::PoseStamped p = pose;
    const double dx = pose.pose.position.x - prediction.x;
    const double dy = pose.pose.position.y - prediction.y;
    p.pose.position.x = c * dx + s * dy;
    p.pose.position.y = -s * dx + c * dy;
    p.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
      tf2::getYaw(pose.pose.orientation) - prediction.yaw);
    predicted_path.poses.push_back(p);
  }

  // Drop the path behind the predicted pose, as the controller server does for the robot pose.
  // Search only as far as the robot could have moved so we don't skip ahead to a later portion
  // of the path, and keep at least 2 points to retain the end-of-path direction.
  if (predicted_path.poses.size() > 2) {
    const double search_dist = 2.0 * hypot(prediction.x, prediction.y) + 0.5;
    auto search_end = nav2_util::geometry_utils::first_after_integrated_distance(
      predicted_path.poses.begin(), predicted_path.poses.end(), search_dist);
    auto closest = nav2_util::geometry_utils::min_by(
      predicted_path.poses.begin(), search_end,
      [](const geometry_msgs::msg::PoseStamped & ps) {
        return hypot(ps.pose.position.x, ps.pose.position.y);
      });
    if (closest == std::prev(predicted_path.poses.end())) {
      closest = std::prev(closest);
    }
    predicted_path.poses.erase(predicted_path.poses.begin(), closest);
  }

  return predicted_path;
}

void RegulatedPurePursuitController::newPathReceived(
  const nav_msgs::msg::Path & /*raw_global_path*/)
{
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    params_->max_linear_vel = params_->base_max_linear_vel;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      params_->max_linear_vel = params_->base_max_linear_vel * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      params_->max_linear_vel = speed_limit;
    }
  }
}

void RegulatedPurePursuitController::reset()
{
  cancelling_ = false;
  finished_cancelling_ = false;
  last_command_velocity_ = geometry_msgs::msg::Twist();
  last_desired_velocity_ = geometry_msgs::msg::Twist();
  latency_compensator_->reset();
}
}  // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav2_core::Controller)
