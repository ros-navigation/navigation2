// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

namespace mppi
{

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);

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
  getParam(s.base_constraints.az_max, "az_max", 3.5f);
  getParam(s.sampling_std.vx, "vx_std", 0.2f);
  getParam(s.sampling_std.vy, "vy_std", 0.2f);
  getParam(s.sampling_std.wz, "wz_std", 0.4f);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);

  s.base_constraints.ax_max = fabs(s.base_constraints.ax_max);
  if (s.base_constraints.ax_min > 0.0) {
    s.base_constraints.ax_min = -1.0 * s.base_constraints.ax_min;
    RCLCPP_WARN(
      logger_,
      "Sign of the parameter ax_min is incorrect, consider setting it negative.");
  }

  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));

  s.constraints = s.base_constraints;

  setMotionModel(motion_model_name);
  parameters_handler_->addPostCallback([this]() {reset();});

  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
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

void Optimizer::reset()
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_[0] = {0.0f, 0.0f, 0.0f};
  control_history_[1] = {0.0f, 0.0f, 0.0f};
  control_history_[2] = {0.0f, 0.0f, 0.0f};
  control_history_[3] = {0.0f, 0.0f, 0.0f};

  settings_.constraints = settings_.base_constraints;

  costs_.setZero(settings_.batch_size);
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  RCLCPP_INFO(logger_, "Optimizer reset");
}

bool Optimizer::isHolonomic() const
{
  return motion_model_->isHolonomic();
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::Pose & goal,
  nav2_core::GoalChecker * goal_checker)
{
  prepare(robot_pose, robot_speed, plan, goal, goal_checker);

  do {
    optimize();
  } while (fallback(critics_data_.fail_flag));

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
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

  reset();

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
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  path_ = utils::toTensor(plan);
  costs_.setZero();
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
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  float max_delta_vx = s.model_dt * s.constraints.ax_max;
  float min_delta_vx = s.model_dt * s.constraints.ax_min;
  float max_delta_vy = s.model_dt * s.constraints.ay_max;
  float max_delta_wz = s.model_dt * s.constraints.az_max;
  float vx_last = utils::clamp(s.constraints.vx_min, s.constraints.vx_max, control_sequence_.vx(0));
  float wz_last = utils::clamp(-s.constraints.wz, s.constraints.wz, control_sequence_.wz(0));
  control_sequence_.vx(0) = vx_last;
  control_sequence_.wz(0) = wz_last;
  float vy_last = 0;
  if (isHolonomic()) {
    vy_last = utils::clamp(-s.constraints.vy, s.constraints.vy, control_sequence_.vy(0));
    control_sequence_.vy(0) = vy_last;
  }

  for (unsigned int i = 1; i != control_sequence_.vx.size(); i++) {
    float & vx_curr = control_sequence_.vx(i);
    vx_curr = utils::clamp(s.constraints.vx_min, s.constraints.vx_max, vx_curr);
    vx_curr = utils::clamp(vx_last + min_delta_vx, vx_last + max_delta_vx, vx_curr);
    vx_last = vx_curr;

    float & wz_curr = control_sequence_.wz(i);
    wz_curr = utils::clamp(-s.constraints.wz, s.constraints.wz, wz_curr);
    wz_curr = utils::clamp(wz_last - max_delta_wz, wz_last + max_delta_wz, wz_curr);
    wz_last = wz_curr;

    if (isHolonomic()) {
      float & vy_curr = control_sequence_.vy(i);
      vy_curr = utils::clamp(-s.constraints.vy, s.constraints.vy, vy_curr);
      vy_curr = utils::clamp(vy_last - max_delta_vy, vy_last + max_delta_vy, vy_curr);
      vy_last = vy_curr;
    }
  }

  motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state) const
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

  float last_yaw = initial_yaw;
  for(size_t i = 0; i != n_size; i++) {
    last_yaw += wz(i) * settings_.model_dt;
    traj_yaws(i) = last_yaw;
  }

  Eigen::ArrayXf yaw_cos(traj_yaws.rows());
  Eigen::ArrayXf yaw_sin(traj_yaws.rows());
  yaw_cos = traj_yaws.cos();
  yaw_sin = traj_yaws.sin();
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
  for(size_t i = 0; i != n_size; i++) {
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

  Eigen::ArrayXf last_x = Eigen::ArrayXf::Constant(trajectories.x.rows(),
    state.pose.pose.position.x);
  Eigen::ArrayXf last_y = Eigen::ArrayXf::Constant(trajectories.y.rows(),
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

void Optimizer::updateControlSequence()
{
  const bool is_holo = isHolonomic();
  auto & s = settings_;

  auto vx_T = control_sequence_.vx.transpose();
  auto wz_T = control_sequence_.wz.transpose();
  auto bounded_noises_vx = state_.cvx.rowwise() - vx_T;
  auto bounded_noises_wz = state_.cwz.rowwise() - wz_T;
  const float gamma_vx = s.gamma / (s.sampling_std.vx * s.sampling_std.vx);
  const float gamma_wz = s.gamma / (s.sampling_std.wz * s.sampling_std.wz);
  costs_ += (gamma_vx * (bounded_noises_vx.rowwise() * vx_T).rowwise().sum()).eval();
  costs_ += (gamma_wz * (bounded_noises_wz.rowwise() * wz_T).rowwise().sum()).eval();

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

void Optimizer::setMotionModel(const std::string & model)
{
  if (model == "DiffDrive") {
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    motion_model_ = std::make_shared<OmniMotionModel>();
  } else if (model == "Ackermann") {
    motion_model_ = std::make_shared<AckermannMotionModel>(parameters_handler_, name_);
  } else {
    throw nav2_core::ControllerException(
            std::string(
              "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann"));
  }
  motion_model_->initialize(settings_.constraints, settings_.model_dt);
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
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
