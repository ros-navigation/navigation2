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

// Closed-loop benchmark of RPP on a simulated robot with execution latency, lag, gain error
// and deadbands. Drives the real controller plugin through swath (boustrophedon) and S-curve
// paths, with and without latency compensation, and reports tracking metrics.
//
// Environment variables:
//   RPP_LATENCY_BENCHMARK_FULL=1   run the full plant grid rather than the CI subset
//   RPP_BENCH_CSV_DIR=<dir>        write a summary and per-run trajectories as CSV

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "angles/angles.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/path_utils.hpp"
#include "path_utils/path_utils.hpp"
#include "tf2/utils.hpp"

using nav2_regulated_pure_pursuit_controller::AxisModel;

namespace
{

constexpr double kControlDt = 0.05;
constexpr int kPlantSubsteps = 5;
constexpr double kStartTime = 100.0;
constexpr double kLocalPlanLength = 4.0;
constexpr double kSettleTime = 2.0;

struct PlantModel
{
  std::string name;
  AxisModel linear;
  AxisModel angular;
  double linear_noise{0.0};
  double angular_noise{0.0};
};

/**
 * Hammerstein unicycle: command -> deadband -> gain -> saturation -> delay -> first-order lag,
 * with optional velocity noise on the realized motion (e.g. legged gait sway).
 */
class LaggedUnicycle
{
public:
  LaggedUnicycle(const PlantModel & model, double x, double y, double yaw, unsigned int seed)
  : model_(model), x_(x), y_(y), yaw_(yaw), rng_(seed) {}

  void command(double t, double v, double w)
  {
    commands_.push_back({t, v, w});
    const double oldest =
      t - std::max(model_.linear.delay, model_.angular.delay) - 1.0;
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

    const double target_v = std::clamp(model_.linear.steadyState(cmd_v), -1.0, 1.0);
    const double target_w = std::clamp(model_.angular.steadyState(cmd_w), -2.5, 2.5);
    const double new_v = v_ + alpha(model_.linear.time_constant, dt) * (target_v - v_);
    const double new_w = w_ + alpha(model_.angular.time_constant, dt) * (target_w - w_);

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
  static double alpha(double tau, double dt)
  {
    return tau > 0.0 ? 1.0 - std::exp(-dt / tau) : 1.0;
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
};

struct ControllerConfig
{
  std::string name;
  bool enabled{false};
  bool lead{false};
  bool invert{false};
  double lookahead_dist{0.6};
  // Multiplicative error of the controller's delay model relative to the true plant
  double delay_error{1.0};
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
  double start_overshoot{0.0};
  double goal_overshoot{0.0};
  int goal_rotation_reversals{0};
};

nav_msgs::msg::Path makeSwathPath()
{
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.orientation.w = 1.0;
  auto path = path_utils::generate_path(
    start, 0.05, {
      std::make_unique<path_utils::Straight>(6.0),
      std::make_unique<path_utils::LeftTurnAround>(1.0),
      std::make_unique<path_utils::Straight>(6.0),
      std::make_unique<path_utils::RightTurnAround>(1.0),
      std::make_unique<path_utils::Straight>(6.0),
      std::make_unique<path_utils::LeftTurnAround>(0.6),
      std::make_unique<path_utils::Straight>(6.0)});
  // Require a final rotation in place to the goal heading
  auto & goal = path.poses.back().pose;
  goal.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    tf2::getYaw(goal.orientation) + M_PI_2);
  return path;
}

nav_msgs::msg::Path makeSCurvePath()
{
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.orientation.w = 1.0;
  return path_utils::generate_path(
    start, 0.05, {
      std::make_unique<path_utils::Straight>(1.0),
      std::make_unique<path_utils::LeftTurn>(1.5),
      std::make_unique<path_utils::RightTurn>(1.5),
      std::make_unique<path_utils::RightTurn>(1.0),
      std::make_unique<path_utils::LeftTurn>(1.0),
      std::make_unique<path_utils::LeftTurn>(0.7),
      std::make_unique<path_utils::RightTurn>(0.7),
      std::make_unique<path_utils::RightTurn>(0.7),
      std::make_unique<path_utils::LeftTurn>(0.7),
      std::make_unique<path_utils::Straight>(1.0)});
}

PlantModel plant(
  const std::string & name, double delay, double tau,
  double ang_gain = 1.0, double lin_db = 0.0, double ang_db = 0.0,
  double lin_noise = 0.0, double ang_noise = 0.0)
{
  PlantModel p;
  p.name = name;
  p.linear.delay = delay;
  p.linear.time_constant = tau;
  p.angular.delay = delay;
  p.angular.time_constant = tau;
  p.angular.gain = ang_gain;
  p.linear.deadband = lin_db;
  p.angular.deadband = ang_db;
  p.linear_noise = lin_noise;
  p.angular_noise = ang_noise;
  return p;
}

class LatencyBenchmark : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("rpp_latency_benchmark");
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

    const std::string n = "FollowPath.";
    auto declare = [](const std::string & name, const rclcpp::ParameterValue & value) {
        node_->declare_parameter(name, value);
      };
    declare(n + "max_linear_vel", rclcpp::ParameterValue(0.5));
    declare(n + "lookahead_dist", rclcpp::ParameterValue(0.6));
    declare(n + "use_collision_detection", rclcpp::ParameterValue(false));
    declare(n + "use_cost_regulated_linear_velocity_scaling", rclcpp::ParameterValue(false));
    declare(n + "use_rotate_to_heading", rclcpp::ParameterValue(true));
    declare(n + "rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
    declare(n + "max_angular_accel", rclcpp::ParameterValue(3.2));
    declare(n + "regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.9));

    controller_ =
      std::make_shared<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
    controller_->configure(node_, "FollowPath", tf_, costmap_);
    controller_->activate();

    checker_ = std::make_shared<nav2_controller::SimpleGoalChecker>();
    checker_->initialize(node_, "goal_checker", costmap_);
  }

  static void TearDownTestSuite()
  {
    controller_->deactivate();
    controller_->cleanup();
    controller_.reset();
    checker_.reset();
    costmap_.reset();
    tf_.reset();
    node_.reset();
  }

  static void setModel(const PlantModel & p, const ControllerConfig & c)
  {
    const std::string n = "FollowPath.latency_compensation.";
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter(n + "enabled", c.enabled),
      rclcpp::Parameter(n + "use_lead_compensation", c.lead),
      rclcpp::Parameter(n + "invert_actuation", c.invert),
      rclcpp::Parameter(n + "linear_delay", p.linear.delay * c.delay_error),
      rclcpp::Parameter(n + "angular_delay", p.angular.delay * c.delay_error),
      rclcpp::Parameter(n + "linear_time_constant", p.linear.time_constant),
      rclcpp::Parameter(n + "angular_time_constant", p.angular.time_constant),
      rclcpp::Parameter(n + "linear_gain", p.linear.gain),
      rclcpp::Parameter(n + "angular_gain", p.angular.gain),
      rclcpp::Parameter(n + "linear_deadband", p.linear.deadband),
      rclcpp::Parameter(n + "angular_deadband", p.angular.deadband),
      rclcpp::Parameter("FollowPath.lookahead_dist", c.lookahead_dist),
    };
    for (const auto & result : node_->set_parameters(params)) {
      ASSERT_TRUE(result.successful) << result.reason;
    }
  }

  static RunResult run(
    const Scenario & scenario, const PlantModel & p, const ControllerConfig & c,
    const std::string & csv_path = "")
  {
    setModel(p, c);
    controller_->reset();
    checker_->reset();
    // Each run restarts simulated time
    tf_->clear();

    const auto & plan = scenario.plan;
    const auto & start = plan.poses.front().pose.position;
    LaggedUnicycle robot(p, start.x, start.y, scenario.start_yaw, 42);

    geometry_msgs::msg::PoseStamped goal = plan.poses.back();
    goal.header.frame_id = "map";
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);
    const double path_start_yaw = tf2::getYaw(plan.poses.front().pose.orientation);
    const double path_length = nav2_util::geometry_utils::calculate_path_length(plan);
    const int max_steps = static_cast<int>(4.0 * path_length / 0.5 / kControlDt) + 400;

    std::ofstream csv;
    if (!csv_path.empty()) {
      csv.open(csv_path);
      csv << "t,x,y,yaw,v,w,cmd_v,cmd_w,cte\n";
    }

    RunResult result;
    size_t closest = 0;
    size_t plan_start = 0;
    double sum_sq_cte = 0.0, traveled = 0.0;
    int samples = 0;
    double start_err_sign = angles::shortest_angular_distance(scenario.start_yaw, path_start_yaw);
    double goal_err_sign = 0.0;
    double last_goal_cmd_w = 0.0;
    double prev_x = start.x, prev_y = start.y;

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
        traveled += std::hypot(robot.x() - prev_x, robot.y() - prev_y);
        prev_x = robot.x();
        prev_y = robot.y();

        // Overshoot of the initial rotation onto the path heading
        if (traveled < 0.3) {
          const double e = angles::shortest_angular_distance(robot.yaw(), path_start_yaw);
          result.start_overshoot =
            std::max(result.start_overshoot, -std::copysign(1.0, start_err_sign) * e);
        }

        // Overshoot and reversals of the final rotation onto the goal heading
        const double dist_to_goal = std::hypot(
          robot.x() - goal.pose.position.x, robot.y() - goal.pose.position.y);
        if (dist_to_goal < 0.25) {
          const double e = angles::shortest_angular_distance(robot.yaw(), goal_yaw);
          if (goal_err_sign == 0.0) {
            goal_err_sign = std::copysign(1.0, e);
          }
          result.goal_overshoot = std::max(result.goal_overshoot, -goal_err_sign * e);
          if (std::fabs(cmd.angular.z) > 0.05) {
            if (last_goal_cmd_w * cmd.angular.z < 0.0) {
              result.goal_rotation_reversals++;
            }
            last_goal_cmd_w = cmd.angular.z;
          }
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
      tf_msg.transform.rotation =
        nav2_util::geometry_utils::orientationAroundZAxis(robot.yaw());
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
      if (checker_->isGoalReached(robot_pose.pose, goal.pose, measured, local_plan)) {
        result.success = true;
        break;
      }

      geometry_msgs::msg::Twist cmd;
      try {
        cmd = controller_->computeVelocityCommands(
          robot_pose, measured, checker_.get(), local_plan, goal).twist;
      } catch (const std::exception & ex) {
        ADD_FAILURE() << "Controller threw: " << ex.what();
        break;
      }

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
          t_end + s * kControlDt + i * kControlDt / kPlantSubsteps,
          kControlDt / kPlantSubsteps);
      }
      sample(t_end + (s + 1) * kControlDt, zero);
    }

    result.rms_cte = samples > 0 ? std::sqrt(sum_sq_cte / samples) : 0.0;
    result.final_xy_error = std::hypot(
      robot.x() - goal.pose.position.x, robot.y() - goal.pose.position.y);
    result.final_yaw_error =
      std::fabs(angles::shortest_angular_distance(robot.yaw(), goal_yaw));
    return result;
  }

  static void printHeader()
  {
    printf(
      "%-8s %-14s %-17s %4s %8s %8s %7s %7s %7s %8s %8s %4s\n",
      "path", "plant", "controller", "ok", "rms_cte", "max_cte", "time", "xy_err",
      "yaw_err", "start_os", "goal_os", "rev");
  }

  static void printRow(
    const Scenario & s, const PlantModel & p, const ControllerConfig & c, const RunResult & r,
    std::ofstream & summary)
  {
    printf(
      "%-8s %-14s %-17s %4s %8.3f %8.3f %7.1f %7.3f %7.3f %8.3f %8.3f %4d\n",
      s.name.c_str(), p.name.c_str(), c.name.c_str(), r.success ? "yes" : "NO", r.rms_cte,
      r.max_cte, r.time, r.final_xy_error, r.final_yaw_error, r.start_overshoot,
      r.goal_overshoot, r.goal_rotation_reversals);
    if (summary.is_open()) {
      summary << s.name << "," << p.name << "," << c.name << "," << r.success << "," <<
        r.rms_cte << "," << r.max_cte << "," << r.time << "," << r.final_xy_error << "," <<
        r.final_yaw_error << "," << r.start_overshoot << "," << r.goal_overshoot << "," <<
        r.goal_rotation_reversals << "\n";
    }
  }

  static nav2::LifecycleNode::SharedPtr node_;
  static nav2::TransformBuffer::SharedPtr tf_;
  static std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  static std::shared_ptr<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>
  controller_;
  static std::shared_ptr<nav2_controller::SimpleGoalChecker> checker_;
};

nav2::LifecycleNode::SharedPtr LatencyBenchmark::node_;
nav2::TransformBuffer::SharedPtr LatencyBenchmark::tf_;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> LatencyBenchmark::costmap_;
std::shared_ptr<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>
LatencyBenchmark::controller_;
std::shared_ptr<nav2_controller::SimpleGoalChecker> LatencyBenchmark::checker_;

// RPP's default lookahead, and tighter ones which latency compensation makes usable
const ControllerConfig kBaseline{"baseline", false, false, false, 0.6};
const ControllerConfig kPredict{"predict", true, false, false, 0.6};
const ControllerConfig kComp{"predict+invert", true, false, true, 0.6};
const ControllerConfig kBaselineTight{"baseline_L0.3", false, false, false, 0.3};
const ControllerConfig kCompTight{"comp_L0.3", true, false, true, 0.3};
// Under-estimating the delay is the harmful direction of model error
const ControllerConfig kCompDelayError{"comp_L0.45_d-30%", true, false, true, 0.45, 0.7};
const ControllerConfig kCompLead{"comp+lead", true, true, true, 0.6};

}  // namespace

TEST_F(LatencyBenchmark, swathsAndSCurves)
{
  const bool full = std::getenv("RPP_LATENCY_BENCHMARK_FULL") != nullptr;
  const char * csv_dir = std::getenv("RPP_BENCH_CSV_DIR");

  std::vector<Scenario> scenarios = {
    {"swath", makeSwathPath(), M_PI_2, true},
    {"s_curve", makeSCurvePath(), 0.0, false},
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
    plants.push_back(plant("d0.25_tau0.20", 0.25, 0.2));
    plants.push_back(plant("d0.40_tau0.40", 0.4, 0.4));
  }
  // Legged-like: large delay and lag, under-turning, deadbands and gait sway
  plants.push_back(plant("legged", 0.3, 0.3, 0.8, 0.05, 0.1, 0.03, 0.05));

  std::vector<ControllerConfig> configs = {
    kBaseline, kPredict, kComp, kBaselineTight, kCompTight, kCompDelayError};
  if (full) {
    configs.push_back(kCompLead);
  }

  std::ofstream summary;
  if (csv_dir) {
    summary.open(std::string(csv_dir) + "/summary.csv");
    summary << "path,plant,controller,success,rms_cte,max_cte,time,final_xy_error," <<
      "final_yaw_error,start_overshoot,goal_overshoot,goal_rotation_reversals\n";
  }

  printHeader();
  for (const auto & s : scenarios) {
    for (const auto & p : plants) {
      std::vector<RunResult> r;
      for (const auto & c : configs) {
        std::string csv_path;
        if (csv_dir) {
          csv_path = std::string(csv_dir) + "/" + s.name + "__" + p.name + "__" + c.name + ".csv";
        }
        r.push_back(run(s, p, c, csv_path));
        printRow(s, p, c, r.back(), summary);
      }

      const auto & base = r[0];
      const auto & predict = r[1];
      const auto & comp = r[2];
      const auto & base_tight = r[3];
      const auto & comp_tight = r[4];
      const auto & comp_delay_error = r[5];
      const std::string id = s.name + " " + p.name;

      if (p.linear.isIdentity() && p.angular.isIdentity()) {
        // Compensation with an ideal model must not change behavior
        EXPECT_TRUE(base.success) << id;
        EXPECT_NEAR(predict.rms_cte, base.rms_cte, 1e-9) << id;
        EXPECT_NEAR(comp.rms_cte, base.rms_cte, 1e-9) << id;
        EXPECT_NEAR(comp_tight.rms_cte, base_tight.rms_cte, 1e-9) << id;
        continue;
      }

      if (p.linear.delay < 0.25 || p.linear.time_constant < 0.2) {
        // Small or pure delays can offset pure pursuit's corner cutting, so there is no
        // consistent tracking benefit to assert. See the README for these results.
        continue;
      }

      // With significant delay and lag, compensation must complete every path, stop rotations
      // on the goal heading rather than past it, and allow a tighter lookahead that tracks
      // the path far better than baseline RPP at either lookahead, even with model error.
      for (const auto * result : {&predict, &comp, &comp_tight, &comp_delay_error}) {
        EXPECT_TRUE(result->success) << id;
      }
      if (s.rotate_at_goal) {
        EXPECT_LE(comp.goal_overshoot, std::max(0.02, 0.5 * base.goal_overshoot)) << id;
      }
      const double best_baseline = std::min(base.rms_cte, base_tight.rms_cte);
      EXPECT_LT(comp_tight.rms_cte, 0.5 * best_baseline) << id;
      EXPECT_LT(comp_delay_error.rms_cte, 0.8 * best_baseline) << id;
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
