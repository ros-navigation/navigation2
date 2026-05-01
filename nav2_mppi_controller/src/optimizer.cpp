// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2025 Open Navigation LLC
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

#include "nav2_mppi_controller/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace mppi
{

void Optimizer::initialize(
  nav2::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;
  tf_buffer_ = tf_buffer;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);

  // This may throw an exception if not valid and fail initialization
  nav2::declare_parameter_if_not_declared(
    node, name_ + ".TrajectoryValidator.plugin",
    rclcpp::ParameterValue("mppi::DefaultOptimalTrajectoryValidator"));
  std::string validator_plugin_type = nav2::get_plugin_type_param(
    node, name_ + ".TrajectoryValidator");
  validator_loader_ = std::make_unique<pluginlib::ClassLoader<OptimalTrajectoryValidator>>(
    "nav2_mppi_controller", "mppi::OptimalTrajectoryValidator");
  trajectory_validator_ = validator_loader_->createUniqueInstance(validator_plugin_type);
  trajectory_validator_->initialize(
    parent_, name_ + ".TrajectoryValidator",
    costmap_ros_, parameters_handler_, tf_buffer, settings_);
  RCLCPP_INFO(logger_, "Loaded trajectory validator plugin: %s", validator_plugin_type.c_str());

  accel_pub_ = node->create_publisher<geometry_msgs::msg::AccelStamped>(
    name_ + "/cmd_acceleration", 10);

  reset();
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");

  // Reject dynamic updates to kinematic params when speed limit is active
  auto kinematic_guard = [this](
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result) {
      if (isSpeedLimitActive()) {
        result.successful = false;
        if (!result.reason.empty()) {
          result.reason += "\n";
        }
        result.reason += "Rejected dynamic update to '" + param.get_name() +
          "': speed limit is active. Clear the speed limit first.";
      }
    };

  const std::vector<std::string> kinematic_params = {
    "vx_max", "vx_min", "vy_max", "wz_max"};
  for (const auto & p : kinematic_params) {
    parameters_handler_->addPreCallback(name_ + "." + p, kinematic_guard);
  }

  getParam(s.model_dt, "model_dt", 0.05f);
  getParam(s.time_steps, "time_steps", 56);
  getParam(s.batch_size, "batch_size", 1000);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.3f);
  getParam(s.gamma, "gamma", 0.015f);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5f);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35f);
  getParam(s.base_constraints.vy, "vy_max", 0.5f);
  getParam(s.base_constraints.wz, "wz_max", 1.9f);
  getParam(s.base_constraints.ax_max, "ax_max", 3.0f);
  getParam(s.base_constraints.ax_min, "ax_min", -3.0f);
  getParam(s.base_constraints.ay_max, "ay_max", 3.0f);
  getParam(s.base_constraints.ay_min, "ay_min", -3.0f);
  getParam(s.base_constraints.az_max, "az_max", 3.5f);
  getParam(s.sampling_std.vx, "vx_std", 0.2f);
  getParam(s.sampling_std.vy, "vy_std", 0.2f);
  getParam(s.sampling_std.wz, "wz_std", 0.4f);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);
  getParam(s.open_loop, "open_loop", false);
  getParam(s.sgf_order, "sgf_order", 2);
  if (s.sgf_order < 1 || s.sgf_order > 2) {
    RCLCPP_WARN(logger_, "sgf_order must be 1 or 2, defaulting to 2");
    s.sgf_order = 2;
  }

  s.base_constraints.ax_max = fabs(s.base_constraints.ax_max);
  if (s.base_constraints.ax_min > 0.0) {
    s.base_constraints.ax_min = -1.0 * s.base_constraints.ax_min;
    RCLCPP_WARN(
      logger_,
      "Sign of the parameter ax_min is incorrect, consider setting it negative.");
  }

  if (s.base_constraints.ay_min > 0.0) {
    s.base_constraints.ay_min = -1.0 * s.base_constraints.ay_min;
    RCLCPP_WARN(
      logger_,
      "Sign of the parameter ay_min is incorrect, consider setting it negative.");
  }

  getParam(motion_model_name, "motion_model", std::string("diff_drive"));

  s.constraints = s.base_constraints;

  setMotionModel(motion_model_name);
  parameters_handler_->addPostCallback([this]() {reset();});

  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  s.controller_period = static_cast<float>(1.0 / controller_frequency);
  setOffset(s.controller_period);
}

void Optimizer::setOffset(double controller_period)
{
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < settings_.model_dt) {
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control sequence "
      "shifting is ON");
    settings_.shift_control_sequence = true;
  } else {
    throw nav2_core::ControllerException(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset(bool reset_dynamic_speed_limits)
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_[0] = {0.0f, 0.0f, 0.0f};
  control_history_[1] = {0.0f, 0.0f, 0.0f};
  control_history_[2] = {0.0f, 0.0f, 0.0f};
  control_history_[3] = {0.0f, 0.0f, 0.0f};

  last_command_vel_ = geometry_msgs::msg::Twist();
  last_command_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  smoothed_ax_ = 0.0;
  smoothed_az_ = 0.0;

  if (reset_dynamic_speed_limits) {
    settings_.constraints = settings_.base_constraints;
  }

  costs_.setZero(settings_.batch_size);
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
  trajectory_validator_->initialize(
    parent_, name_ + ".TrajectoryValidator",
    costmap_ros_, parameters_handler_, tf_buffer_, settings_);

  RCLCPP_INFO(logger_, "Optimizer reset");
}

bool Optimizer::isHolonomic() const
{
  return motion_model_->isHolonomic();
}

bool Optimizer::isSpeedLimitActive() const
{
  // Speed limit is active when current constraints differ from base constraints.
  // This occurs when setSpeedLimit() has modified the velocity/acceleration limits.
  const auto & base = settings_.base_constraints;
  const auto & curr = settings_.constraints;
  return base.vx_max != curr.vx_max ||
         base.vx_min != curr.vx_min ||
         base.vy != curr.vy ||
         base.wz != curr.wz;
}

std::tuple<geometry_msgs::msg::TwistStamped, Eigen::ArrayXXf> Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::Pose & goal,
  nav2_core::GoalChecker * goal_checker)
{
  prepare(robot_pose, robot_speed, plan, goal, goal_checker);
  Eigen::ArrayXXf optimal_trajectory;
  bool trajectory_valid = true;

  do {
    optimize();
    optimal_trajectory = getOptimizedTrajectory();
    switch (trajectory_validator_->validateTrajectory(
        optimal_trajectory, control_sequence_, robot_pose, robot_speed, plan, goal))
    {
      case mppi::ValidationResult::SOFT_RESET:
        trajectory_valid = false;
        RCLCPP_WARN(logger_, "Soft reset triggered by trajectory validator");
        break;
      case mppi::ValidationResult::FAILURE:
        throw nav2_core::NoValidControl(
                "Trajectory validator failed to validate trajectory, hard reset triggered.");
      case mppi::ValidationResult::SUCCESS:
      default:
        trajectory_valid = true;
        break;
    }
  } while (fallback(critics_data_.fail_flag || !trajectory_valid));

  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  // Publish acceleration between last command and current first control
  if (accel_pub_ && last_command_time_.nanoseconds() != 0) {
    double dt = (rclcpp::Time(plan.header.stamp) - last_command_time_).seconds();
    if (dt > 0.0) {
      geometry_msgs::msg::AccelStamped accel_msg;
      accel_msg.header.stamp = plan.header.stamp;
      accel_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
      constexpr double alpha = 0.3;
      smoothed_ax_ = alpha * (control.twist.linear.x - last_command_vel_.linear.x) / dt +
        (1.0 - alpha) * smoothed_ax_;
      smoothed_az_ = alpha * (control.twist.angular.z - last_command_vel_.angular.z) / dt +
        (1.0 - alpha) * smoothed_az_;
      accel_msg.accel.linear.x = smoothed_ax_;
      accel_msg.accel.angular.z = smoothed_az_;
      accel_pub_->publish(accel_msg);
    }
  }

  last_command_vel_ = control.twist;
  last_command_time_ = plan.header.stamp;

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return std::make_tuple(control, optimal_trajectory);
}

void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    updateControlSequence();
  }
}

bool Optimizer::fallback(bool fail)
{
  static size_t counter = 0;

  if (!fail) {
    counter = 0;
    return false;
  }

  reset(false /*Don't reset zone-based speed limits after fallback*/);

  if (++counter > settings_.retry_attempt_limit) {
    counter = 0;
    throw nav2_core::NoValidControl("Optimizer fail to compute path");
  }

  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::Pose & goal,
  nav2_core::GoalChecker * goal_checker)
{
  if (settings_.open_loop) {
    state_.speed = last_command_vel_;
  } else {
    // Predict state one controller_period forward toward the last command to compensate
    // for the latency between measurement and when this command will take effect.
    // Clamp to physically achievable range so prediction never exceeds dynamics.
    const auto & c = settings_.constraints;
    const double dt = settings_.controller_period;
    state_.speed = robot_speed;
    state_.speed.linear.x = std::clamp(
      last_command_vel_.linear.x,
      robot_speed.linear.x + dt * c.ax_min,
      robot_speed.linear.x + dt * c.ax_max);
    state_.speed.angular.z = std::clamp(
      last_command_vel_.angular.z,
      robot_speed.angular.z - dt * c.az_max,
      robot_speed.angular.z + dt * c.az_max);
    if (isHolonomic()) {
      state_.speed.linear.y = std::clamp(
        last_command_vel_.linear.y,
        robot_speed.linear.y + dt * c.ay_min,
        robot_speed.linear.y + dt * c.ay_max);
    }
  }

  state_.pose = robot_pose;
  state_.local_path_length = nav2_util::geometry_utils::calculate_path_length(plan);
  path_ = utils::toTensor(plan);
  costs_.setZero(settings_.batch_size);
  goal_ = goal;

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
}

void Optimizer::shiftControlSequence()
{
  auto size = control_sequence_.vx.size();
  utils::shiftColumnsByOnePlace(control_sequence_.vx, -1);
  utils::shiftColumnsByOnePlace(control_sequence_.wz, -1);
  control_sequence_.vx(size - 1) = control_sequence_.vx(size - 2);
  control_sequence_.wz(size - 1) = control_sequence_.wz(size - 2);

  if (isHolonomic()) {
    utils::shiftColumnsByOnePlace(control_sequence_.vy, -1);
    control_sequence_.vy(size - 1) = control_sequence_.vy(size - 2);
  }
}

void Optimizer::generateNoisedTrajectories()
{
  applyControlSequenceInterIterationConstraints();
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

void Optimizer::applyControlSequenceInterIterationConstraints()
{
  // Enforce t=0 to be dynamically feasible from the current speed for inter-iteration feasibility
  // Re-centers the distribution at t=0, but still applied in a information theoretic sound way

  // Use controller_period for t=0 to realistically model the physics intra-iteration
  auto & s = settings_;
  float first_dt = s.controller_period;
  float max_delta_vx = first_dt * s.constraints.ax_max;
  float min_delta_vx = first_dt * s.constraints.ax_min;
  float max_delta_vy = first_dt * s.constraints.ay_max;
  float min_delta_vy = first_dt * s.constraints.ay_min;
  float max_delta_wz = first_dt * s.constraints.az_max;

  float speed_vx = static_cast<float>(state_.speed.linear.x);
  float speed_wz = static_cast<float>(state_.speed.angular.z);
  if (s.shift_control_sequence) {
    // When shifting, vx(0) is not sent and represents 'now'
    // so that vx(1), the sent command, needs to be only one step away.
    control_sequence_.vx(0) = speed_vx;
    control_sequence_.wz(0) = speed_wz;
    if (isHolonomic()) {
      control_sequence_.vy(0) = static_cast<float>(state_.speed.linear.y);
    }
  } else {
    // When not shifting, vx(0) is the sent command, clamp to the feasible envelope
    control_sequence_.vx(0) = utils::clampVelocityByAccel(
      speed_vx, control_sequence_.vx(0), min_delta_vx, max_delta_vx);
    control_sequence_.wz(0) = utils::clampVelocityByAccel(
      speed_wz, control_sequence_.wz(0), -max_delta_wz, max_delta_wz);
    if (isHolonomic()) {
      float speed_vy = static_cast<float>(state_.speed.linear.y);
      control_sequence_.vy(0) = utils::clampVelocityByAccel(
        speed_vy, control_sequence_.vy(0), min_delta_vy, max_delta_vy);
    }
  }
}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  // Apply constraints to set the optimal control sequence within bounds
  motion_model_->applyConstraints(control_sequence_);

  // Use controller_period for t=0 to realistically model physical limits
  float first_dt = s.controller_period;
  float max_delta_vx = first_dt * s.constraints.ax_max;
  float min_delta_vx = first_dt * s.constraints.ax_min;
  float max_delta_vy = first_dt * s.constraints.ay_max;
  float min_delta_vy = first_dt * s.constraints.ay_min;
  float max_delta_wz = first_dt * s.constraints.az_max;

  // Initialize as the current speed to create inter-iteration dynamic feasibility
  float vx_last = static_cast<float>(state_.speed.linear.x);
  float wz_last = static_cast<float>(state_.speed.angular.z);
  float vy_last = isHolonomic() ? static_cast<float>(state_.speed.linear.y) : 0.0f;

  // When shifting, vx(0) is "now" and not sent. Pin it so vx(1), the sent command,
  // is exactly one constraint step from current speed when shift_control_sequence
  if (s.shift_control_sequence) {
    control_sequence_.vx(0) = vx_last;
    control_sequence_.wz(0) = wz_last;
    if (isHolonomic()) {
      control_sequence_.vy(0) = vy_last;
    }
  }

  for (unsigned int i = 0; i != control_sequence_.vx.size(); i++) {
    // After first timestep, switch to MPC model_dt for intra-iteration feasibility
    if (i == 1) {
      max_delta_vx = s.model_dt * s.constraints.ax_max;
      min_delta_vx = s.model_dt * s.constraints.ax_min;
      max_delta_vy = s.model_dt * s.constraints.ay_max;
      min_delta_vy = s.model_dt * s.constraints.ay_min;
      max_delta_wz = s.model_dt * s.constraints.az_max;
    }

    float & vx_curr = control_sequence_.vx(i);
    vx_curr = utils::clamp(s.constraints.vx_min, s.constraints.vx_max, vx_curr);
    vx_curr = utils::clampVelocityByAccel(vx_last, vx_curr, min_delta_vx, max_delta_vx);
    vx_last = vx_curr;

    float & wz_curr = control_sequence_.wz(i);
    wz_curr = utils::clamp(-s.constraints.wz, s.constraints.wz, wz_curr);
    wz_curr = utils::clampVelocityByAccel(wz_last, wz_curr, -max_delta_wz, max_delta_wz);
    wz_last = wz_curr;

    if (isHolonomic()) {
      float & vy_curr = control_sequence_.vy(i);
      vy_curr = utils::clamp(-s.constraints.vy, s.constraints.vy, vy_curr);
      vy_curr = utils::clampVelocityByAccel(vy_last, vy_curr, min_delta_vy, max_delta_vy);
      vy_last = vy_curr;
    }
  }

  // Apply again to ensure accel constraints don't violate specialty limits
  motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(models::State & state) const
{
  state.vx.col(0) = static_cast<float>(state.speed.linear.x);
  state.wz.col(0) = static_cast<float>(state.speed.angular.z);

  if (isHolonomic()) {
    state.vy.col(0) = static_cast<float>(state.speed.linear.y);
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  Eigen::Array<float, Eigen::Dynamic, 3> & trajectory,
  const Eigen::ArrayXXf & sequence) const
{
  float initial_yaw = static_cast<float>(tf2::getYaw(state_.pose.pose.orientation));

  const auto vx = sequence.col(0);
  const auto wz = sequence.col(1);

  auto traj_x = trajectory.col(0);
  auto traj_y = trajectory.col(1);
  auto traj_yaws = trajectory.col(2);

  const size_t n_size = traj_yaws.size();
  if (n_size == 0) {
    return;
  }

  float last_yaw = initial_yaw;
  for (size_t i = 0; i != n_size; i++) {
    last_yaw += wz(i) * settings_.model_dt;
    traj_yaws(i) = last_yaw;
  }

  Eigen::ArrayXf yaw_cos = traj_yaws.cos();
  Eigen::ArrayXf yaw_sin = traj_yaws.sin();
  utils::shiftColumnsByOnePlace(yaw_cos, 1);
  utils::shiftColumnsByOnePlace(yaw_sin, 1);
  yaw_cos(0) = cosf(initial_yaw);
  yaw_sin(0) = sinf(initial_yaw);

  auto dx = (vx * yaw_cos).eval();
  auto dy = (vx * yaw_sin).eval();

  if (isHolonomic()) {
    auto vy = sequence.col(2);
    dx = (dx - vy * yaw_sin).eval();
    dy = (dy + vy * yaw_cos).eval();
  }

  float last_x = state_.pose.pose.position.x;
  float last_y = state_.pose.pose.position.y;
  for (size_t i = 0; i != n_size; i++) {
    last_x += dx(i) * settings_.model_dt;
    last_y += dy(i) * settings_.model_dt;
    traj_x(i) = last_x;
    traj_y(i) = last_y;
  }
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  auto initial_yaw = static_cast<float>(tf2::getYaw(state.pose.pose.orientation));
  const size_t n_cols = trajectories.yaws.cols();

  Eigen::ArrayXf last_yaws = Eigen::ArrayXf::Constant(trajectories.yaws.rows(), initial_yaw);
  for (size_t i = 0; i != n_cols; i++) {
    last_yaws += state.wz.col(i) * settings_.model_dt;
    trajectories.yaws.col(i) = last_yaws;
  }

  Eigen::ArrayXXf yaw_cos = trajectories.yaws.cos();
  Eigen::ArrayXXf yaw_sin = trajectories.yaws.sin();
  utils::shiftColumnsByOnePlace(yaw_cos, 1);
  utils::shiftColumnsByOnePlace(yaw_sin, 1);
  yaw_cos.col(0) = cosf(initial_yaw);
  yaw_sin.col(0) = sinf(initial_yaw);

  auto dx = (state.vx * yaw_cos).eval();
  auto dy = (state.vx * yaw_sin).eval();

  if (isHolonomic()) {
    dx -= state.vy * yaw_sin;
    dy += state.vy * yaw_cos;
  }

  Eigen::ArrayXf last_x = Eigen::ArrayXf::Constant(
    trajectories.x.rows(),
    state.pose.pose.position.x);
  Eigen::ArrayXf last_y = Eigen::ArrayXf::Constant(
    trajectories.y.rows(),
    state.pose.pose.position.y);

  for (size_t i = 0; i != n_cols; i++) {
    last_x += dx.col(i) * settings_.model_dt;
    last_y += dy.col(i) * settings_.model_dt;
    trajectories.x.col(i) = last_x;
    trajectories.y.col(i) = last_y;
  }
}

Eigen::ArrayXXf Optimizer::getOptimizedTrajectory()
{
  const bool is_holo = isHolonomic();
  Eigen::ArrayXXf sequence = Eigen::ArrayXXf(settings_.time_steps, is_holo ? 3 : 2);
  Eigen::Array<float, Eigen::Dynamic, 3> trajectories =
    Eigen::Array<float, Eigen::Dynamic, 3>(settings_.time_steps, 3);

  sequence.col(0) = control_sequence_.vx;
  sequence.col(1) = control_sequence_.wz;

  if (is_holo) {
    sequence.col(2) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return trajectories;
}

const models::ControlSequence & Optimizer::getOptimalControlSequence()
{
  return control_sequence_;
}

void Optimizer::updateControlSequence()
{
  const bool is_holo = isHolonomic();
  auto & s = settings_;

  auto vx_T = control_sequence_.vx.transpose();
  auto bounded_noises_vx = state_.cvx.rowwise() - vx_T;
  const float gamma_vx = s.gamma / (s.sampling_std.vx * s.sampling_std.vx);
  costs_ += (gamma_vx * (bounded_noises_vx.rowwise() * vx_T).rowwise().sum()).eval();

  if (s.sampling_std.wz > 0.0f) {
    auto wz_T = control_sequence_.wz.transpose();
    auto bounded_noises_wz = state_.cwz.rowwise() - wz_T;
    const float gamma_wz = s.gamma / (s.sampling_std.wz * s.sampling_std.wz);
    costs_ += (gamma_wz * (bounded_noises_wz.rowwise() * wz_T).rowwise().sum()).eval();
  }

  if (is_holo) {
    auto vy_T = control_sequence_.vy.transpose();
    auto bounded_noises_vy = state_.cvy.rowwise() - vy_T;
    const float gamma_vy = s.gamma / (s.sampling_std.vy * s.sampling_std.vy);
    costs_ += (gamma_vy * (bounded_noises_vy.rowwise() * vy_T).rowwise().sum()).eval();
  }

  auto costs_normalized = costs_ - costs_.minCoeff();
  const float inv_temp = 1.0f / s.temperature;
  auto softmaxes = (-inv_temp * costs_normalized).exp().eval();
  softmaxes /= softmaxes.sum();

  auto softmax_mat = softmaxes.matrix();
  control_sequence_.vx = state_.cvx.transpose().matrix() * softmax_mat;
  control_sequence_.wz = state_.cwz.transpose().matrix() * softmax_mat;

  if (is_holo) {
    control_sequence_.vy = state_.cvy.transpose().matrix() * softmax_mat;
  }

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);

  applyControlSequenceConstraints();
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  if (isHolonomic()) {
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string & motion_model_name)
{
  auto node = parent_.lock();
  const std::string plugin_ns = name_ + "." + motion_model_name;
  std::string plugin_type;
  motion_model_loader_ =
    std::make_unique<pluginlib::ClassLoader<MotionModel>>(
    "nav2_mppi_controller", "mppi::MotionModel");

  try {
    plugin_type = nav2::get_plugin_type_param(node, plugin_ns);
    motion_model_ = motion_model_loader_->createSharedInstance(plugin_type);
    motion_model_->initialize(parameters_handler_, plugin_ns);
    motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
  } catch (const pluginlib::PluginlibException & ex) {
    throw nav2_core::ControllerException(
            std::string("Failed to load motion model plugin '") + motion_model_name +
            "': " + ex.what());
  }

  RCLCPP_INFO(logger_, "Loaded motion model plugin: %s", plugin_type.c_str());
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
  motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
