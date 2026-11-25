// Copyright (c) 2026 Open Navigation LLC
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

// Closed-loop benchmark of MPPI on a simulated robot with execution delay, lag, gain error
// and deadbands. Drives the real controller plugin through swath (boustrophedon), S-curve and
// straight paths, with and without the robot's delay modeled, and reports tracking metrics.
//
// Environment variables:
//   MPPI_LATENCY_BENCHMARK_FULL=1   run the full plant grid rather than the default subset
//   MPPI_BENCH_REPEATS=<n>          repeats of each run, as MPPI's sampling is stochastic
//   MPPI_BENCH_PLANTS=<a,b,...>     only run plants with any of the names, where a trailing *
//                                   matches any ending
//   MPPI_BENCH_CONFIGS=<a,b,...>    only run controller configurations likewise
//   MPPI_BENCH_PATHS=<a,b,...>      only run paths likewise
//   MPPI_BENCH_CSV_DIR=<dir>        write a summary, every run's metrics and the first repeat's
//                                   trajectories as CSV
//   MPPI_BENCH_PLANT_ACCEL=1        the robot's velocity also changes within the configured
//                                   acceleration limits, as a real robot's would
//   MPPI_BENCH_SOFT_RESET=1         apply a 50% speed limit while driving, then update a parameter
//                                   while it is active, which resets the optimizer

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "angles/angles.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/path_utils.hpp"
#include "tf2/utils.hpp"

namespace
{

constexpr double kControlDt = 0.05;
constexpr int kPlantSubsteps = 5;
constexpr double kStartTime = 100.0;
constexpr double kLocalPlanLength = 2.0;
constexpr double kSettleTime = 2.0;
constexpr double kXYTolerance = 0.25;
constexpr double kYawTolerance = 0.25;
constexpr int kMaxConsecutiveFailures = 20;
constexpr int kPredictionSteps = 20;

struct AxisResponse
{
  double delay{0.0};
  double time_constant{0.0};
  double gain{1.0};
  double deadband{0.0};

  double actuate(double cmd) const {return std::fabs(cmd) < deadband ? 0.0 : gain * cmd;}
};

struct PlantModel
{
  std::string name;
  AxisResponse linear;
  AxisResponse angular;
  double linear_noise{0.0};
  double angular_noise{0.0};

  bool isIdeal() const
  {
    return linear.delay == 0.0 && linear.time_constant == 0.0 && angular.delay == 0.0 &&
           angular.time_constant == 0.0 && linear.gain == 1.0 && angular.gain == 1.0 &&
           linear.deadband == 0.0 && angular.deadband == 0.0;
  }
};

/**
 * @brief Range of velocities reachable in one step from a velocity, accelerating away from zero
 * by up to accel and toward zero by up to decel, including through zero within the step
 */
std::pair<double, double> reachable(double v, double accel, double decel, double dt)
{
  const double a = accel * dt, d = decel * dt;
  if (v >= 0.0) {
    const double lo = v >= d ? v - d : -accel * (dt - v / decel);
    return {lo, v + a};
  }
  const double hi = -v >= d ? v + d : accel * (dt + v / decel);
  return {v - a, hi};
}

/**
 * Hammerstein unicycle: command -> deadband -> gain -> delay -> first-order lag, with optional
 * velocity noise on the realized motion (e.g. legged gait sway).
 */
class LaggedUnicycle
{
public:
  LaggedUnicycle(const PlantModel & model, double x, double y, double yaw, unsigned int seed)
  : model_(model), x_(x), y_(y), yaw_(yaw), rng_(seed) {}

  /**
   * @brief Limit the robot's velocity changes to acceleration limits, as a real robot's are
   */
  void setAccelLimits(double ax_max, double ax_min, double az_max)
  {
    accel_limited_ = true;
    ax_max_ = ax_max;
    ax_min_ = ax_min;
    az_max_ = az_max;
  }

  void command(double t, double v, double w)
  {
    commands_.push_back({t, v, w});
    const double oldest = t - std::max(model_.linear.delay, model_.angular.delay) - 1.0;
    while (commands_.size() > 2 && commands_[1].t < oldest) {
      commands_.erase(commands_.begin());
    }
  }

  void step(double t, double dt)
  {
    double cmd_v = 0.0, cmd_w = 0.0;
    for (auto it = commands_.rbegin(); it != commands_.rend(); ++it) {
      if (it->t + model_.linear.delay <= t + 1e-9) {
        cmd_v = it->v;
        break;
      }
    }
    for (auto it = commands_.rbegin(); it != commands_.rend(); ++it) {
      if (it->t + model_.angular.delay <= t + 1e-9) {
        cmd_w = it->w;
        break;
      }
    }

    double new_v = lag(v_, model_.linear.actuate(cmd_v), model_.linear.time_constant, dt);
    double new_w = lag(w_, model_.angular.actuate(cmd_w), model_.angular.time_constant, dt);
    if (accel_limited_) {
      const auto [v_lo, v_hi] = reachable(v_, ax_max_, -ax_min_, dt);
      const auto [w_lo, w_hi] = reachable(w_, az_max_, az_max_, dt);
      new_v = std::clamp(new_v, v_lo, v_hi);
      new_w = std::clamp(new_w, w_lo, w_hi);
    }

    std::normal_distribution<double> noise(0.0, 1.0);
    const bool moving = std::fabs(new_v) > 1e-3 || std::fabs(new_w) > 1e-3;
    meas_v_ = new_v + (moving ? model_.linear_noise * noise(rng_) : 0.0);
    meas_w_ = new_w + (moving ? model_.angular_noise * noise(rng_) : 0.0);

    const double v_avg = 0.5 * (v_ + new_v) + (meas_v_ - new_v);
    const double w_avg = 0.5 * (w_ + new_w) + (meas_w_ - new_w);
    const double yaw_mid = yaw_ + 0.5 * w_avg * dt;
    x_ += v_avg * std::cos(yaw_mid) * dt;
    y_ += v_avg * std::sin(yaw_mid) * dt;
    yaw_ = angles::normalize_angle(yaw_ + w_avg * dt);
    v_ = new_v;
    w_ = new_w;
  }

  double x() const {return x_;}
  double y() const {return y_;}
  double yaw() const {return yaw_;}

  geometry_msgs::msg::Twist measuredVelocity() const
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = meas_v_;
    t.angular.z = meas_w_;
    return t;
  }

private:
  static double lag(double vel, double target, double tau, double dt)
  {
    return tau > 0.0 ? vel + (1.0 - std::exp(-dt / tau)) * (target - vel) : target;
  }

  struct Command
  {
    double t, v, w;
  };

  PlantModel model_;
  double x_, y_, yaw_;
  double v_{0.0}, w_{0.0};
  double meas_v_{0.0}, meas_w_{0.0};
  std::vector<Command> commands_;
  std::mt19937 rng_;
  bool accel_limited_{false};
  double ax_max_{0.0}, ax_min_{0.0}, az_max_{0.0};
};

/**
 * Goal checker matching nav2_controller::SimpleGoalChecker's default, stateful behavior
 */
class BenchmarkGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const nav2::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}

  void reset() override {xy_reached_ = false;}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity, const nav_msgs::msg::Path & plan) override
  {
    if (!isGoalXYReached(query, goal, velocity, plan)) {
      return false;
    }
    return std::fabs(
      angles::shortest_angular_distance(
        tf2::getYaw(query.orientation), tf2::getYaw(goal.orientation))) <= kYawTolerance;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist &,
    double & path_length_tolerance) override
  {
    pose_tolerance.position.x = kXYTolerance;
    pose_tolerance.position.y = kXYTolerance;
    pose_tolerance.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(kYawTolerance);
    path_length_tolerance = 1.0;
    return true;
  }

  bool isGoalXYReached(
    const geometry_msgs::msg::Pose & query, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist &, const nav_msgs::msg::Path &) override
  {
    if (!xy_reached_) {
      xy_reached_ = std::hypot(
        query.position.x - goal.position.x, query.position.y - goal.position.y) <= kXYTolerance;
    }
    return xy_reached_;
  }

private:
  bool xy_reached_{false};
};

/**
 * MPPI controller exposing its optimal trajectory, to measure how well it predicts the robot
 */
class InspectableController : public nav2_mppi_controller::MPPIController
{
public:
  std::tuple<geometry_msgs::msg::TwistStamped, Eigen::ArrayXXf> evalControl(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    return optimizer_.evalControl(pose, speed, plan, goal.pose, goal_checker);
  }
};

struct ControllerConfig
{
  std::string name;
  // Fraction of the true plant delay given to the controller's delay model
  double delay_model_fraction{0.0};
  // Acceleration limits, as a robot which cannot accelerate quickly would be configured
  double ax_max{3.0};
  double ax_min{-3.0};
  double az_max{3.5};
  double vx_std{0.2};
  double wz_std{0.4};
  int time_steps{56};
  bool open_loop{false};
};

struct Scenario
{
  std::string name;
  nav_msgs::msg::Path plan;
  double start_yaw;
  bool rotate_at_goal;
};

struct RunResult
{
  bool success{false};
  double rms_cte{0.0};
  double max_cte{0.0};
  double time{0.0};
  double final_xy_error{0.0};
  double final_yaw_error{0.0};
  double goal_overshoot{0.0};
  int failures{0};
  // Mean error of the robot's position predicted by the optimal trajectory, kPredictionSteps ahead
  double prediction_error{0.0};
  // RMS of the second difference of the commands, per axis, which measures oscillation
  // rather than acceleration
  double jitter_v{0.0};
  double jitter_w{0.0};
  // Commands exceeding the acceleration limits from the previous command
  int accel_violations{0};
  // Time from the start until the measured speed first reaches 90% of vx_max, or -1
  double time_to_speed{-1.0};
  // Mean computation time of a control cycle
  double cycle_ms{0.0};
  // Hash of every command, to check whether a change leaves the behavior bit-identical
  uint64_t cmd_hash{0};
  // With MPPI_BENCH_SOFT_RESET: commands above the speed limit after the parameter update, and how
  // far the command at the update exceeds the acceleration limits, in per-step velocity changes
  int limit_lost{0};
  double reset_jump{0.0};
};

void hashCombine(uint64_t & hash, double value)
{
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (int i = 0; i < 8; ++i) {
    hash ^= (bits >> (8 * i)) & 0xff;
    hash *= 1099511628211ull;
  }
}

// Straight segments and arcs (positive angle turns left), as in the RPP benchmark
nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & segments)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  constexpr double spacing = 0.05;
  double x = 0.0, y = 0.0, yaw = 0.0;
  auto add = [&]() {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      path.poses.push_back(p);
    };
  add();
  for (const auto & [length_or_radius, angle] : segments) {
    if (angle == 0.0) {
      const int n = static_cast<int>(length_or_radius / spacing);
      for (int i = 0; i < n; ++i) {
        x += spacing * std::cos(yaw);
        y += spacing * std::sin(yaw);
        add();
      }
    } else {
      const double r = length_or_radius;
      const int n = std::max(1, static_cast<int>(std::fabs(angle) * r / spacing));
      const double dyaw = angle / n;
      for (int i = 0; i < n; ++i) {
        const double chord = 2.0 * r * std::sin(std::fabs(dyaw) / 2.0);
        x += chord * std::cos(yaw + dyaw / 2.0);
        y += chord * std::sin(yaw + dyaw / 2.0);
        yaw += dyaw;
        add();
      }
    }
  }
  return path;
}

nav_msgs::msg::Path makeSwathPath()
{
  auto path = makePath(
    {{6.0, 0.0}, {1.0, M_PI}, {6.0, 0.0}, {1.0, -M_PI}, {6.0, 0.0}, {0.6, M_PI}, {6.0, 0.0}});
  // Require a final rotation in place to the goal heading
  auto & goal = path.poses.back().pose;
  goal.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    tf2::getYaw(goal.orientation) + M_PI_2);
  return path;
}

nav_msgs::msg::Path makeSCurvePath()
{
  return makePath(
    {{1.0, 0.0}, {1.5, M_PI_2}, {1.5, -M_PI_2}, {1.0, -M_PI_2}, {1.0, M_PI_2},
      {0.7, M_PI_2}, {0.7, -M_PI_2}, {0.7, -M_PI_2}, {0.7, M_PI_2}, {1.0, 0.0}});
}

PlantModel plant(
  const std::string & name, double delay, double tau,
  double ang_gain = 1.0, double lin_db = 0.0, double ang_db = 0.0,
  double lin_noise = 0.0, double ang_noise = 0.0)
{
  PlantModel p;
  p.name = name;
  p.linear = {delay, tau, 1.0, lin_db};
  p.angular = {delay, tau, ang_gain, ang_db};
  p.linear_noise = lin_noise;
  p.angular_noise = ang_noise;
  return p;
}

class MPPILatencyBenchmark : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("mppi_latency_benchmark");
    node_->declare_parameter("controller_frequency", rclcpp::ParameterValue(1.0 / kControlDt));
    const std::string n = "FollowPath.";
    auto declare = [](const std::string & name, const rclcpp::ParameterValue & value) {
        node_->declare_parameter(name, value);
      };
    // The Nav2 bringup configuration
    declare(n + "time_steps", rclcpp::ParameterValue(56));
    declare(n + "model_dt", rclcpp::ParameterValue(kControlDt));
    declare(n + "batch_size", rclcpp::ParameterValue(1000));
    declare(n + "vx_max", rclcpp::ParameterValue(0.5));
    declare(n + "vx_std", rclcpp::ParameterValue(0.2));
    declare(n + "wz_std", rclcpp::ParameterValue(0.4));
    declare(n + "vx_min", rclcpp::ParameterValue(-0.35));
    declare(n + "wz_max", rclcpp::ParameterValue(1.9));
    declare(n + "visualize", rclcpp::ParameterValue(false));
    declare(n + "motion_model", rclcpp::ParameterValue(std::string("DiffDrive")));
    declare(
      n + "DiffDrive.plugin", rclcpp::ParameterValue(std::string("mppi::DiffDriveMotionModel")));
    declare(
      n + "critics", rclcpp::ParameterValue(
        std::vector<std::string>{"ConstraintCritic", "CostCritic", "GoalCritic",
          "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
          "PreferForwardCritic"}));
    declare(n + "ConstraintCritic.cost_weight", rclcpp::ParameterValue(4.0));
    declare(n + "GoalCritic.cost_weight", rclcpp::ParameterValue(5.0));
    declare(n + "GoalCritic.threshold_to_consider", rclcpp::ParameterValue(1.4));
    declare(n + "GoalAngleCritic.cost_weight", rclcpp::ParameterValue(3.0));
    declare(n + "GoalAngleCritic.threshold_to_consider", rclcpp::ParameterValue(0.5));
    declare(n + "PreferForwardCritic.cost_weight", rclcpp::ParameterValue(5.0));
    declare(n + "PreferForwardCritic.threshold_to_consider", rclcpp::ParameterValue(0.5));
    declare(n + "CostCritic.cost_weight", rclcpp::ParameterValue(3.81));
    declare(n + "CostCritic.consider_footprint", rclcpp::ParameterValue(false));
    declare(n + "PathAlignCritic.cost_weight", rclcpp::ParameterValue(14.0));
    declare(n + "PathAlignCritic.threshold_to_consider", rclcpp::ParameterValue(0.5));
    declare(n + "PathAlignCritic.offset_from_furthest", rclcpp::ParameterValue(20));
    declare(n + "PathFollowCritic.cost_weight", rclcpp::ParameterValue(5.0));
    declare(n + "PathFollowCritic.offset_from_furthest", rclcpp::ParameterValue(5));
    declare(n + "PathFollowCritic.threshold_to_consider", rclcpp::ParameterValue(1.4));
    declare(n + "PathAngleCritic.cost_weight", rclcpp::ParameterValue(2.0));
    declare(n + "PathAngleCritic.offset_from_furthest", rclcpp::ParameterValue(4));
    declare(n + "PathAngleCritic.threshold_to_consider", rclcpp::ParameterValue(0.5));
    declare(n + "model_delay_vx", rclcpp::ParameterValue(0.0));
    declare(n + "model_delay_wz", rclcpp::ParameterValue(0.0));
    declare(n + "ax_max", rclcpp::ParameterValue(3.0));
    declare(n + "ax_min", rclcpp::ParameterValue(-3.0));
    declare(n + "az_max", rclcpp::ParameterValue(3.5));
    declare(n + "open_loop", rclcpp::ParameterValue(false));

    tf_ = nav2::create_transform_buffer(node_);
    // Transforms are set synchronously by the benchmark loop
    tf_->setUsingDedicatedThread(true);

    // An empty costmap large enough to contain the test paths
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("bench_costmap");
    costmap_->declare_parameter("width", rclcpp::ParameterValue(80));
    costmap_->declare_parameter("height", rclcpp::ParameterValue(80));
    costmap_->declare_parameter("origin_x", rclcpp::ParameterValue(-40.0));
    costmap_->declare_parameter("origin_y", rclcpp::ParameterValue(-40.0));
    costmap_->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
    costmap_->declare_parameter(
      "plugins", rclcpp::ParameterValue(std::vector<std::string>{}));
    rclcpp_lifecycle::State state;
    costmap_->on_configure(state);
  }

  /**
   * @brief Start a new controller, so that its random sampling is independent of earlier runs
   */
  static void startController(const PlantModel & p, const ControllerConfig & c)
  {
    stopController();
    setModel(p, c);
    controller_ = std::make_shared<InspectableController>();
    controller_->configure(node_, "FollowPath", tf_, costmap_);
    controller_->activate();
  }

  static void stopController()
  {
    if (controller_) {
      controller_->deactivate();
      controller_->cleanup();
      controller_.reset();
    }
  }

  static void TearDownTestSuite()
  {
    stopController();
    costmap_.reset();
    tf_.reset();
    node_.reset();
  }

  static void setModel(const PlantModel & p, const ControllerConfig & c)
  {
    const std::string n = "FollowPath.";
    for (const auto & result : node_->set_parameters(
      {
        rclcpp::Parameter(n + "model_delay_vx", p.linear.delay * c.delay_model_fraction),
        rclcpp::Parameter(n + "model_delay_wz", p.angular.delay * c.delay_model_fraction),
        rclcpp::Parameter(n + "ax_max", c.ax_max),
        rclcpp::Parameter(n + "ax_min", c.ax_min),
        rclcpp::Parameter(n + "az_max", c.az_max),
        rclcpp::Parameter(n + "vx_std", c.vx_std),
        rclcpp::Parameter(n + "wz_std", c.wz_std),
        rclcpp::Parameter(n + "time_steps", c.time_steps),
        rclcpp::Parameter(n + "open_loop", c.open_loop),
      }))
    {
      ASSERT_TRUE(result.successful) << result.reason;
    }
  }

  static RunResult run(
    const Scenario & scenario, const PlantModel & p, const ControllerConfig & c,
    const std::string & csv_path = "")
  {
    controller_->reset();
    const bool soft_reset = std::getenv("MPPI_BENCH_SOFT_RESET") != nullptr;
    constexpr int kSpeedLimitCycle = 100, kParamUpdateCycle = 140;
    static int param_updates = 0;
    if (soft_reset) {
      controller_->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
    }
    BenchmarkGoalChecker checker;
    tf_->clear();

    const auto & plan = scenario.plan;
    const auto & start = plan.poses.front().pose.position;
    LaggedUnicycle robot(p, start.x, start.y, scenario.start_yaw, 42);
    if (std::getenv("MPPI_BENCH_PLANT_ACCEL")) {
      robot.setAccelLimits(c.ax_max, c.ax_min, c.az_max);
    }

    geometry_msgs::msg::PoseStamped goal = plan.poses.back();
    goal.header.frame_id = "map";
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);
    const double path_length = nav2_util::geometry_utils::calculate_path_length(plan);
    const int max_steps = static_cast<int>(4.0 * path_length / 0.5 / kControlDt) + 400;

    std::ofstream csv;
    if (!csv_path.empty()) {
      csv.open(csv_path);
      csv << "t,x,y,yaw,v,w,cmd_v,cmd_w,cte\n";
    }

    RunResult result;
    size_t closest = 0, plan_start = 0;
    double sum_sq_cte = 0.0, goal_err_sign = 0.0;
    int samples = 0, consecutive_failures = 0;
    // Predicted positions of the robot, by the control step they are predicted for
    std::deque<std::tuple<int, double, double>> predictions;
    double sum_prediction_error = 0.0;
    int prediction_samples = 0;

    // Command quality: the previous two commands, whether the previous one was a published zero
    // after a failure, and the accumulated oscillation and computation time
    double prev_v = 0.0, prev_w = 0.0, prev2_v = 0.0, prev2_w = 0.0;
    bool prev_failed = false;
    int cmd_count = 0, jitter_samples = 0;
    double sum_sq_jit_v = 0.0, sum_sq_jit_w = 0.0, sum_cycle_ms = 0.0;
    result.cmd_hash = 1469598103934665603ull;

    auto sample = [&](double t, const geometry_msgs::msg::Twist & cmd) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = robot.x();
        pose.position.y = robot.y();
        auto search = nav2_util::distance_from_path(plan, pose, closest, 1.0);
        closest = search.closest_segment_index;
        const double cte = std::fabs(search.distance);
        sum_sq_cte += cte * cte;
        result.max_cte = std::max(result.max_cte, cte);
        samples++;

        // Overshoot of the final rotation onto the goal heading
        if (std::hypot(robot.x() - goal.pose.position.x, robot.y() - goal.pose.position.y) <
          kXYTolerance)
        {
          const double e = angles::shortest_angular_distance(robot.yaw(), goal_yaw);
          if (goal_err_sign == 0.0) {
            goal_err_sign = std::copysign(1.0, e);
          }
          result.goal_overshoot = std::max(result.goal_overshoot, -goal_err_sign * e);
        }

        if (csv.is_open()) {
          auto vel = robot.measuredVelocity();
          csv << t << "," << robot.x() << "," << robot.y() << "," << robot.yaw() << "," <<
            vel.linear.x << "," << vel.angular.z << "," << cmd.linear.x << "," <<
            cmd.angular.z << "," << search.distance << "\n";
        }
      };

    int k = 0;
    for (; k < max_steps; ++k) {
      const double t = kStartTime + k * kControlDt;
      rclcpp::Time stamp(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = "map";
      tf_msg.child_frame_id = "base_link";
      tf_msg.transform.translation.x = robot.x();
      tf_msg.transform.translation.y = robot.y();
      tf_msg.transform.rotation = nav2_util::geometry_utils::orientationAroundZAxis(robot.yaw());
      tf_->setTransform(tf_msg, "benchmark");

      geometry_msgs::msg::PoseStamped robot_pose;
      robot_pose.header.frame_id = "map";
      robot_pose.header.stamp = stamp;
      robot_pose.pose.position.x = robot.x();
      robot_pose.pose.position.y = robot.y();
      robot_pose.pose.orientation = tf_msg.transform.rotation;

      // Local plan from the closest pose, as the controller server's path handler provides
      auto search_end = nav2_util::geometry_utils::first_after_integrated_distance(
        plan.poses.begin() + plan_start, plan.poses.end(), 1.0);
      auto closest_pose = nav2_util::geometry_utils::min_by(
        plan.poses.begin() + plan_start, search_end,
        [&](const geometry_msgs::msg::PoseStamped & ps) {
          return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
        });
      if (closest_pose == std::prev(plan.poses.end())) {
        closest_pose = std::prev(closest_pose);
      }
      plan_start = std::distance(plan.poses.begin(), closest_pose);

      nav_msgs::msg::Path local_plan;
      local_plan.header.frame_id = "map";
      local_plan.header.stamp = stamp;
      double length = 0.0;
      for (size_t i = plan_start; i < plan.poses.size(); ++i) {
        if (i > plan_start) {
          length += nav2_util::geometry_utils::euclidean_distance(
            plan.poses[i - 1], plan.poses[i]);
        }
        if (length > kLocalPlanLength) {
          break;
        }
        local_plan.poses.push_back(plan.poses[i]);
        local_plan.poses.back().header = local_plan.header;
      }
      goal.header.stamp = stamp;

      const auto measured = robot.measuredVelocity();
      if (checker.isGoalReached(robot_pose.pose, goal.pose, measured, local_plan)) {
        result.success = true;
        break;
      }

      while (!predictions.empty() && std::get<0>(predictions.front()) <= k) {
        if (std::get<0>(predictions.front()) == k) {
          sum_prediction_error += std::hypot(
            robot.x() - std::get<1>(predictions.front()),
            robot.y() - std::get<2>(predictions.front()));
          prediction_samples++;
        }
        predictions.pop_front();
      }

      // As the controller server, publish zero on a failure and allow some failure tolerance
      if (result.time_to_speed < 0.0 && measured.linear.x >= 0.9 * 0.5) {
        result.time_to_speed = k * kControlDt;
      }

      if (soft_reset && k == kSpeedLimitCycle) {
        controller_->setSpeedLimit(50.0, true);
      }
      if (soft_reset && k == kParamUpdateCycle) {
        const double weight = 5.0 + 1e-3 * (++param_updates % 2);
        const std::string param = "FollowPath.PathFollowCritic.cost_weight";
        node_->set_parameters({rclcpp::Parameter(param, weight)});
      }

      geometry_msgs::msg::Twist cmd;
      bool failed = false;
      try {
        const auto cycle_start = std::chrono::steady_clock::now();
        auto [cmd_stamped, trajectory] = controller_->evalControl(
          robot_pose, measured, &checker, local_plan, goal);
        sum_cycle_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - cycle_start).count();
        cmd = cmd_stamped.twist;
        consecutive_failures = 0;
        // Each trajectory point is a model_dt (one control step) after the last
        if (trajectory.rows() >= kPredictionSteps) {
          predictions.emplace_back(
            k + kPredictionSteps, trajectory(kPredictionSteps - 1, 0),
            trajectory(kPredictionSteps - 1, 1));
        }
      } catch (const nav2_core::NoValidControl &) {
        failed = true;
        result.failures++;
        if (++consecutive_failures > kMaxConsecutiveFailures) {
          break;
        }
      }

      hashCombine(result.cmd_hash, cmd.linear.x);
      hashCombine(result.cmd_hash, cmd.angular.z);
      if (soft_reset && !failed && k >= kParamUpdateCycle && cmd.linear.x > 0.25 + 1e-3) {
        result.limit_lost++;
      }
      if (soft_reset && !failed && k == kParamUpdateCycle) {
        const auto [lo, hi] = reachable(prev_v, c.ax_max, -c.ax_min, kControlDt);
        result.reset_jump = std::max(0.0, std::max(lo - cmd.linear.x, cmd.linear.x - hi)) /
          (c.ax_max * kControlDt);
      }
      if (!failed && !prev_failed) {
        // Commands are a control period apart, starting from rest
        const auto [v_lo, v_hi] = reachable(prev_v, c.ax_max, -c.ax_min, kControlDt);
        const auto [w_lo, w_hi] = reachable(prev_w, c.az_max, c.az_max, kControlDt);
        constexpr double eps = 1e-4;
        if (cmd.linear.x < v_lo - eps || cmd.linear.x > v_hi + eps ||
          cmd.angular.z < w_lo - eps || cmd.angular.z > w_hi + eps)
        {
          result.accel_violations++;
        }
        if (cmd_count >= 2) {
          const double jv = cmd.linear.x - 2.0 * prev_v + prev2_v;
          const double jw = cmd.angular.z - 2.0 * prev_w + prev2_w;
          sum_sq_jit_v += jv * jv;
          sum_sq_jit_w += jw * jw;
          jitter_samples++;
        }
      }
      prev2_v = prev_v;
      prev2_w = prev_w;
      prev_v = cmd.linear.x;
      prev_w = cmd.angular.z;
      prev_failed = failed;
      cmd_count++;

      robot.command(t, cmd.linear.x, cmd.angular.z);
      sample(t, cmd);
      for (int i = 0; i < kPlantSubsteps; ++i) {
        robot.step(t + i * kControlDt / kPlantSubsteps, kControlDt / kPlantSubsteps);
      }
    }
    result.time = k * kControlDt;

    // Command zero and let the robot settle to measure where it actually stops
    const double t_end = kStartTime + k * kControlDt;
    robot.command(t_end, 0.0, 0.0);
    geometry_msgs::msg::Twist zero;
    const int settle_steps = static_cast<int>(kSettleTime / kControlDt);
    for (int s = 0; s < settle_steps; ++s) {
      for (int i = 0; i < kPlantSubsteps; ++i) {
        robot.step(
          t_end + s * kControlDt + i * kControlDt / kPlantSubsteps, kControlDt / kPlantSubsteps);
      }
      sample(t_end + (s + 1) * kControlDt, zero);
    }

    result.jitter_v = jitter_samples > 0 ? std::sqrt(sum_sq_jit_v / jitter_samples) : 0.0;
    result.jitter_w = jitter_samples > 0 ? std::sqrt(sum_sq_jit_w / jitter_samples) : 0.0;
    const int cycles = cmd_count - result.failures;
    result.cycle_ms = cycles > 0 ? sum_cycle_ms / cycles : 0.0;
    result.rms_cte = samples > 0 ? std::sqrt(sum_sq_cte / samples) : 0.0;
    result.prediction_error =
      prediction_samples > 0 ? sum_prediction_error / prediction_samples : 0.0;
    result.final_xy_error = std::hypot(
      robot.x() - goal.pose.position.x, robot.y() - goal.pose.position.y);
    result.final_yaw_error = std::fabs(angles::shortest_angular_distance(robot.yaw(), goal_yaw));
    return result;
  }

  struct Aggregate
  {
    int successes{0};
    int runs{0};
    double rms_mean{0.0}, rms_min{1e9}, rms_max{0.0};
    double max_cte{0.0}, time{0.0}, goal_overshoot{0.0}, final_yaw_error{0.0};
    double prediction_error{0.0};
    int failures{0};

    void add(const RunResult & r)
    {
      runs++;
      successes += r.success ? 1 : 0;
      rms_mean += (r.rms_cte - rms_mean) / runs;
      rms_min = std::min(rms_min, r.rms_cte);
      rms_max = std::max(rms_max, r.rms_cte);
      max_cte = std::max(max_cte, r.max_cte);
      time += (r.time - time) / runs;
      goal_overshoot = std::max(goal_overshoot, r.goal_overshoot);
      final_yaw_error = std::max(final_yaw_error, r.final_yaw_error);
      failures += r.failures;
      prediction_error += (r.prediction_error - prediction_error) / runs;
    }
  };

  static void printRow(
    const Scenario & s, const PlantModel & p, const ControllerConfig & c, const Aggregate & a,
    std::ofstream & summary)
  {
    printf(
      "%-8s %-14s %-18s %3d/%-3d %7.3f %7.3f %7.3f %8.3f %7.1f %8.3f %8.3f %5d %8.3f\n",
      s.name.c_str(), p.name.c_str(), c.name.c_str(), a.successes, a.runs, a.rms_mean,
      a.rms_min, a.rms_max, a.max_cte, a.time, a.goal_overshoot, a.final_yaw_error, a.failures,
      a.prediction_error);
    fflush(stdout);
    if (summary.is_open()) {
      summary << s.name << "," << p.name << "," << c.name << "," << a.successes << "," <<
        a.runs << "," << a.rms_mean << "," << a.rms_min << "," << a.rms_max << "," <<
        a.max_cte << "," << a.time << "," << a.goal_overshoot << "," << a.final_yaw_error <<
        "," << a.failures << "," << a.prediction_error << "\n";
    }
  }

  static nav2::LifecycleNode::SharedPtr node_;
  static nav2::TransformBuffer::SharedPtr tf_;
  static std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  static std::shared_ptr<InspectableController> controller_;
};

nav2::LifecycleNode::SharedPtr MPPILatencyBenchmark::node_;
nav2::TransformBuffer::SharedPtr MPPILatencyBenchmark::tf_;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> MPPILatencyBenchmark::costmap_;
std::shared_ptr<InspectableController> MPPILatencyBenchmark::controller_;

enum class AccelProfile {DEFAULT, LOW, ASYMMETRIC, ISSUE_6171};

ControllerConfig config(
  const std::string & name, double delay_fraction, AccelProfile profile, bool open_loop = false)
{
  ControllerConfig c;
  c.delay_model_fraction = delay_fraction;
  c.open_loop = open_loop;
  std::string suffix;
  switch (profile) {
    case AccelProfile::LOW:
      suffix = "_low_acc";
      c.ax_max = 0.5;
      c.ax_min = -0.5;
      c.az_max = 1.0;
      break;
    case AccelProfile::ASYMMETRIC:
      // Accelerating slowly but braking hard
      suffix = "_asym_acc";
      c.ax_max = 0.6;
      c.ax_min = -1.7;
      break;
    case AccelProfile::ISSUE_6171:
      // Slow rotational acceleration with large sampling noise and a long horizon
      suffix = "_6171";
      c.ax_max = 1.7;
      c.ax_min = -1.7;
      c.az_max = 0.6061;
      c.vx_std = 0.5;
      c.wz_std = 0.4;
      c.time_steps = 100;
      break;
    case AccelProfile::DEFAULT:
      break;
  }
  c.name = name + suffix + (open_loop ? "_open" : "");
  return c;
}

}  // namespace

/**
 * Keep only the items named in the comma separated environment variable, if it is set, where a
 * trailing * matches any ending
 */
template<typename T>
void filterByName(std::vector<T> & items, const char * variable)
{
  const char * filter = std::getenv(variable);
  if (!filter) {
    return;
  }
  std::vector<std::string> names;
  std::stringstream stream(filter);
  std::string text;
  while (std::getline(stream, text, ',')) {
    names.push_back(text);
  }
  items.erase(
    std::remove_if(
      items.begin(), items.end(),
      [&names](const T & item) {
        return std::none_of(
          names.begin(), names.end(),
          [&item](const std::string & name) {
            if (!name.empty() && name.back() == '*') {
              return item.name.rfind(name.substr(0, name.size() - 1), 0) == 0;
            }
            return item.name == name;
          });
      }),
    items.end());
}

TEST_F(MPPILatencyBenchmark, swathsAndSCurves)
{
  const bool full = std::getenv("MPPI_LATENCY_BENCHMARK_FULL") != nullptr;
  const char * csv_dir = std::getenv("MPPI_BENCH_CSV_DIR");

  std::vector<Scenario> scenarios = {
    {"swath", makeSwathPath(), M_PI_2, true},
    {"s_curve", makeSCurvePath(), 0.0, false},
    // Converging onto a straight path from a heading error
    {"straight", makePath({{10.0, 0.0}}), 0.35, false},
  };

  std::vector<PlantModel> plants;
  if (full) {
    for (double delay : {0.0, 0.1, 0.25, 0.4}) {
      for (double tau : {0.0, 0.2, 0.4}) {
        char name[64];
        snprintf(name, sizeof(name), "d%.2f_tau%.2f", delay, tau);
        plants.push_back(plant(name, delay, tau));
      }
    }
  } else {
    plants.push_back(plant("d0.00_tau0.00", 0.0, 0.0));
    plants.push_back(plant("d0.25_tau0.00", 0.25, 0.0));
    plants.push_back(plant("d0.40_tau0.20", 0.4, 0.2));
  }
  // Legged-like: large delay and lag, under-turning, deadbands and gait sway
  plants.push_back(plant("legged", 0.3, 0.3, 0.8, 0.05, 0.1, 0.03, 0.05));
  plants.push_back(plant("legged_no_db", 0.3, 0.3, 0.8, 0.0, 0.0, 0.03, 0.05));
  filterByName(plants, "MPPI_BENCH_PLANTS");
  filterByName(scenarios, "MPPI_BENCH_PATHS");

  std::vector<ControllerConfig> configs;
  for (auto profile : {AccelProfile::DEFAULT, AccelProfile::LOW, AccelProfile::ASYMMETRIC,
      AccelProfile::ISSUE_6171})
  {
    for (bool open_loop : {false, true}) {
      configs.push_back(config("no_model", 0.0, profile, open_loop));
      configs.push_back(config("delay_model", 1.0, profile, open_loop));
      if (full) {
        // Robustness to under-estimating the delay
        configs.push_back(config("delay_model_-30%", 0.7, profile, open_loop));
      }
    }
  }
  filterByName(configs, "MPPI_BENCH_CONFIGS");
  const char * repeats_env = std::getenv("MPPI_BENCH_REPEATS");
  const int repeats = repeats_env ? std::max(1, std::atoi(repeats_env)) : 3;

  std::ofstream summary, runs_csv;
  if (csv_dir) {
    summary.open(std::string(csv_dir) + "/summary.csv");
    summary << "path,plant,controller,successes,runs,rms_mean,rms_min,rms_max,max_cte,time," <<
      "goal_overshoot,final_yaw_error,failures,prediction_error\n";
    runs_csv.open(std::string(csv_dir) + "/runs.csv");
    runs_csv << "path,plant,controller,repeat,success,rms_cte,max_cte,time,goal_overshoot," <<
      "final_yaw_error,failures,prediction_error,jitter_v,jitter_w,accel_violations," <<
      "time_to_speed,cycle_ms,cmd_hash,limit_lost,reset_jump\n";
  }

  printf(
    "%-8s %-14s %-18s %7s %7s %7s %7s %8s %7s %8s %8s %5s %8s\n", "path", "plant", "controller",
    "ok", "rms", "rms_min", "rms_max", "max_cte", "time", "goal_os", "yaw_err", "fail", "pred_err");
  for (const auto & s : scenarios) {
    for (const auto & p : plants) {
      std::vector<Aggregate> results;
      for (const auto & c : configs) {
        // Repeats sample different noise from the same controller, but each set of repeats is
        // independent of the runs before it
        startController(p, c);
        Aggregate aggregate;
        for (int i = 0; i < repeats; ++i) {
          std::string csv_path;
          if (csv_dir && i == 0) {
            csv_path = std::string(csv_dir) + "/" + s.name + "__" + p.name + "__" + c.name +
              ".csv";
          }
          const auto r = run(s, p, c, csv_path);
          aggregate.add(r);
          if (runs_csv.is_open()) {
            runs_csv << s.name << "," << p.name << "," << c.name << "," << i << "," <<
              r.success << "," << r.rms_cte << "," << r.max_cte << "," << r.time << "," <<
              r.goal_overshoot << "," << r.final_yaw_error << "," << r.failures << "," <<
              r.prediction_error << "," << r.jitter_v << "," << r.jitter_w << "," <<
              r.accel_violations << "," << r.time_to_speed << "," << r.cycle_ms << "," <<
              r.cmd_hash << "," << r.limit_lost << "," << r.reset_jump << "\n";
            runs_csv.flush();
          }
        }
        results.push_back(aggregate);
        printRow(s, p, c, aggregate, summary);
      }

      // Results by configuration name, if it was run
      auto result = [&](const std::string & name) -> const Aggregate * {
          for (size_t i = 0; i < configs.size(); ++i) {
            if (configs[i].name == name) {
              return &results[i];
            }
          }
          return nullptr;
        };
      const std::string id = s.name + " " + p.name;

      // With a delay model, every path is completed
      const auto * delay_model = result("delay_model");
      if (delay_model) {
        EXPECT_EQ(delay_model->successes, delay_model->runs) << id;
      }

      // A robot which accelerates slowly is not held back by its lagging measured velocity, nor
      // modeled as reversing its acceleration after the delay, so tracks swaths far better
      const auto * no_model_low_accel = result("no_model_low_acc");
      const auto * delay_model_low_accel = result("delay_model_low_acc");
      if (no_model_low_accel && delay_model_low_accel && s.name == "swath" &&
        p.linear.delay >= 0.3 && p.linear.time_constant >= 0.2)
      {
        EXPECT_EQ(delay_model_low_accel->successes, delay_model_low_accel->runs) << id;
        EXPECT_LT(delay_model_low_accel->rms_mean, 0.5 * no_model_low_accel->rms_mean) << id;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
