// Copyright (c) 2025 lotusymt
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

// File: collision_monitor_node_bag_metrics.cpp
// @brief Dataset-based test for Collision Monitor. Replays a tiny bag and checks:
//  (1) time-to-stop, (2) hold-stop%, (3) time-to-resume, (4) false-stop% outside window.

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using nav2_msgs::msg::CollisionMonitorState;

struct Sample
{
  double t;          // sim time (seconds)
  double vx;         // cmd_vel.linear.x
  int action;        // last known CM action
};

class MetricsCatcher : public rclcpp::Node {
public:
  MetricsCatcher()
  : rclcpp::Node("cm_metrics_catcher"),
    got_clock_(false), got_costmap_(false),
    last_action_(0)
  {
    // Use bag /clock as time base
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Window where obstacle exists in the bag
    stop_window_start_ = this->declare_parameter<double>("stop_window_start", 3.0);
    stop_window_end_ = this->declare_parameter<double>("stop_window_end", 8.0);


    // Velocity classification thresholds (on the topic we evaluate)
    stop_thresh_ = this->declare_parameter<double>("stop_thresh", 0.02);             // |vx| <= stop => stopped
    resume_thresh_ = this->declare_parameter<double>("resume_thresh", 0.10);         //  vx  >= resume => resumed
    debounce_n_ = this->declare_parameter<int>("debounce_n", 3);                     // need K consecutive samples

    // Acceptable limits for metrics
    max_time_to_stop_ = this->declare_parameter<double>("max_time_to_stop", 0.6);
    min_hold_pct_ = this->declare_parameter<double>("min_hold_pct", 0.90);
    max_time_to_resume_ = this->declare_parameter<double>("max_time_to_resume", 0.6);
    max_false_stop_pct_ = this->declare_parameter<double>("max_false_stop_pct", 0.05);

    // One-shot gates to avoid “startup STOP” counting
    cm_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/local_costmap/costmap", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](const nav2_msgs::msg::Costmap &){
        if (!got_costmap_) {got_costmap_ = true; cm_sub_.reset();}
      });

    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](const rosgraph_msgs::msg::Clock &){
        if (!got_clock_) {got_clock_ = true; clock_sub_.reset();}
      });

    // Output we evaluate (by default: /cmd_vel — change via launch remap if needed)
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(60),
      [this](const geometry_msgs::msg::Twist & msg){
        if (!got_clock_ || !got_costmap_) {return;}
        const double t = this->now().seconds();
        samples_.push_back(Sample{t, msg.linear.x, last_action_});
      });

    // Optional: CM state for debugging / correlation (not used in assertions)
    state_sub_ = this->create_subscription<CollisionMonitorState>(
      "/collision_state", rclcpp::QoS(10),
      [this](const CollisionMonitorState & msg){
        last_action_ = msg.action_type;
      });
  }

  // Spin until we have enough sim time (end + margin)
  void run_and_collect(rclcpp::executors::SingleThreadedExecutor & exec, double margin_s = 2.0)
  {
    // Wait up to 5s WALL for /clock to start
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rclcpp::ok() && !got_clock_) {
      exec.spin_some();
      if (std::chrono::steady_clock::now() > deadline) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Collect until stop window is over + margin (or 30s WALL safety)
    auto wall_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (rclcpp::ok()) {
      exec.spin_some();
      if (this->now().seconds() >= stop_window_end_ + margin_s) {break;}
      if (std::chrono::steady_clock::now() > wall_deadline) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // ----- Metrics we keep -----
  struct Results
  {
    double time_to_stop_s;
    double hold_stop_pct;
    double time_to_resume_s;
    double false_stop_pct;
    bool   have_data;
  };

  Results compute() const
  {
    Results R{};
    R.have_data = !samples_.empty();

    if (!R.have_data) {return R;}

    auto in_range = [&](double t0, double t1){
        std::vector<Sample> out; out.reserve(samples_.size());
        for (const auto & s : samples_) {if (s.t >= t0 && s.t <= t1) {out.push_back(s);}}
        return out;
      };

    const auto stop_seg = in_range(stop_window_start_, stop_window_end_);
    const auto pre_seg = in_range(0.0, std::max(0.0, stop_window_start_ - 0.2));       // small guard band
    const auto post_seg = in_range(stop_window_end_ + 0.2, samples_.back().t);         // small guard band

    // First time we see K-consecutive samples meeting a predicate
    auto first_transition_time = [&](const std::vector<Sample> & seg,
      bool to_stop, double thr, int k)->double {
        int run = 0;
        for (size_t i = 0; i < seg.size(); ++i) {
          const bool ok = to_stop ? (std::fabs(seg[i].vx) <= thr) : (seg[i].vx >= thr);
          run = ok ? (run + 1) : 0;
          if (run >= k) {return seg[i].t;}
        }
        return std::numeric_limits<double>::quiet_NaN();
      };

    // 1) time-to-stop
    const double t_stop_first = first_transition_time(stop_seg, /*to_stop=*/true, stop_thresh_,
      debounce_n_);
    R.time_to_stop_s = std::isnan(t_stop_first) ? std::numeric_limits<double>::infinity() :
      (t_stop_first - stop_window_start_);

    // 2) hold-stop %
    if (!stop_seg.empty()) {
      size_t cnt = 0;
      for (const auto & s : stop_seg) {if (std::fabs(s.vx) <= stop_thresh_) {++cnt;}}
      R.hold_stop_pct = static_cast<double>(cnt) / static_cast<double>(stop_seg.size());
    } else {
      R.hold_stop_pct = 0.0;
    }

    // 3) time-to-resume
    const double t_resume_first = first_transition_time(post_seg, /*to_stop=*/false, resume_thresh_,
      debounce_n_);
    R.time_to_resume_s = std::isnan(t_resume_first) ? std::numeric_limits<double>::infinity() :
      (t_resume_first - stop_window_end_);

    // 4) no-false-stop % outside window
    const size_t clean_total = pre_seg.size() + post_seg.size();
    if (clean_total > 0) {
      auto count_stopped = [&](const std::vector<Sample> & seg){
          return std::count_if(seg.begin(), seg.end(),
                   [&](const Sample & s){return std::fabs(s.vx) <= stop_thresh_;});
        };
      const size_t clean_stopped = count_stopped(pre_seg) + count_stopped(post_seg);
      R.false_stop_pct = static_cast<double>(clean_stopped) / static_cast<double>(clean_total);
    } else {
      R.false_stop_pct = 0.0;
    }

    return R;
  }

  void assert_results(const Results & R)
  {
    ASSERT_TRUE(R.have_data) << "No /cmd_vel samples collected";

    EXPECT_LE(R.time_to_stop_s, max_time_to_stop_) << "time-to-stop too large";
    EXPECT_GE(R.hold_stop_pct, min_hold_pct_) << "hold-stop% too low";
    EXPECT_LE(R.time_to_resume_s, max_time_to_resume_) << "time-to-resume too large";
    EXPECT_LE(R.false_stop_pct, max_false_stop_pct_) << "false-stop% too high";

    RCLCPP_INFO(this->get_logger(),
      "Results: t_stop=%.3fs, hold=%.1f%%, t_resume=%.3fs, false=%.1f%%",
      R.time_to_stop_s, R.hold_stop_pct * 100.0, R.time_to_resume_s, R.false_stop_pct * 100.0);
  }

private:
  // Subscriptions
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr cm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<CollisionMonitorState>::SharedPtr state_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

  // Buffers/state
  std::vector<Sample> samples_;
  bool got_clock_, got_costmap_;
  int  last_action_;

  // Params / thresholds
  double stop_window_start_, stop_window_end_;
  double stop_thresh_, resume_thresh_, max_time_to_stop_, min_hold_pct_;
  double max_time_to_resume_, max_false_stop_pct_;
  int    debounce_n_;
};

TEST(CollisionMonitorNodeBag, TrajectoryAndMetrics)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<MetricsCatcher>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  node->run_and_collect(exec, /*margin_s=*/2.0);

  auto R = node->compute();
  node->assert_results(R);

  rclcpp::shutdown();
}
