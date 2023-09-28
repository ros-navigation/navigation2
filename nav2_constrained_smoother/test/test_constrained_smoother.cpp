// Copyright (c) 2021 RoboTech Vision
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <thread>
#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "angles/angles.h"

#include "nav2_constrained_smoother/constrained_smoother.hpp"

#include "geometry_msgs/msg/pose_array.hpp"

class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  DummyCostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name)
  : CostmapSubscriber(node, topic_name)
  {
    auto costmap = std::make_shared<nav2_msgs::msg::Costmap>();
    costmap->metadata.size_x = 100;
    costmap->metadata.size_y = 100;
    costmap->metadata.resolution = 0.1;
    costmap->metadata.origin.position.x = -5.0;
    costmap->metadata.origin.position.y = -5.0;

    costmap->data.resize(costmap->metadata.size_x * costmap->metadata.size_y, 0);

    // create an obstacle in rect (1.0, -1.0) -> (3.0, -2.0)
    // with inflation of radius 2.0
    double cost_scaling_factor = 1.6;
    double inscribed_radius = 0.3;
    for (int i = 10; i < 60; ++i) {
      for (int j = 40; j < 100; ++j) {
        int dist_x = std::max(0, std::max(60 - j, j - 80));
        int dist_y = std::max(0, std::max(30 - i, i - 40));
        double dist = sqrt(dist_x * dist_x + dist_y * dist_y) * costmap->metadata.resolution;
        unsigned char cost;
        if (dist == 0) {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else if (dist < inscribed_radius) {
          cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        } else {
          double factor =
            exp(
            -1.0 * cost_scaling_factor * (dist - inscribed_radius));
          cost =
            static_cast<unsigned char>((nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        costmap->data[i * costmap->metadata.size_x + j] = cost;
      }
    }

    setCostmap(costmap);
  }

  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    costmap_msg_ = msg;
    costmap_received_ = true;
  }
};

geometry_msgs::msg::Point pointMsg(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

class SmootherTest : public ::testing::Test
{
protected:
  SmootherTest() {SetUp();}
  ~SmootherTest() {}

  void SetUp() override
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "ConstrainedSmootherTestNode", rclcpp::NodeOptions());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    costmap_sub_ =
      std::make_shared<DummyCostmapSubscriber>(
      node_lifecycle_, "costmap_topic");

    path_poses_pub_ = node_lifecycle_->create_publisher<geometry_msgs::msg::PoseArray>(
      "/plan_poses_optimized", 100);
    path_poses_pub_cmp_ = node_lifecycle_->create_publisher<geometry_msgs::msg::PoseArray>(
      "/plan_poses_optimized_cmp", 100);
    path_poses_pub_orig_ = node_lifecycle_->create_publisher<geometry_msgs::msg::PoseArray>(
      "/plan_poses_original", 100);
    costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      node_lifecycle_,
      costmap_sub_->getCostmap().get(), "map", "/costmap", true);

    node_lifecycle_->configure();
    node_lifecycle_->activate();
    path_poses_pub_->on_activate();
    path_poses_pub_cmp_->on_activate();
    path_poses_pub_orig_->on_activate();
    costmap_pub_->on_activate();


    smoother_ = std::make_shared<nav2_constrained_smoother::ConstrainedSmoother>();

    smoother_->configure(
      node_lifecycle_, "SmoothPath", tf_buffer_, costmap_sub_,
      std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>());
    smoother_->activate();

    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 2000000.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.minimum_turning_radius", 0.4));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 30.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 0.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", -1.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_cusp_multiplier", 1.0));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_downsampling_factor", 1));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 1));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.reversing_enabled", true));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_start_orientation", true));
    node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_goal_orientation", true));
    node_lifecycle_->set_parameter(
      rclcpp::Parameter("SmoothPath.optimizer.linear_solver_type", "SPARSE_NORMAL_CHOLESKY"));
    node_lifecycle_->set_parameter(
      rclcpp::Parameter(
        "SmoothPath.cost_check_points",
        std::vector<double>()));
    reloadParams();
  }

  void TearDown() override
  {
    smoother_->deactivate();
    smoother_->cleanup();
    path_poses_pub_->on_deactivate();
    path_poses_pub_cmp_->on_deactivate();
    path_poses_pub_orig_->on_deactivate();
    costmap_pub_->on_deactivate();
    node_lifecycle_->deactivate();
  }

  void reloadParams()
  {
    smoother_->deactivate();
    smoother_->cleanup();
    smoother_->configure(
      node_lifecycle_, "SmoothPath", tf_buffer_, costmap_sub_,
      std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>());
    smoother_->activate();
  }

  bool smoothPath(
    const std::vector<Eigen::Vector3d> & input, std::vector<Eigen::Vector3d> & output,
    bool publish = false, bool cmp = false)
  {
    nav_msgs::msg::Path path;
    path.poses.reserve(input.size());
    for (auto & xya : input) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = xya.x();
      pose.pose.position.y = xya.y();
      pose.pose.position.z = 0;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(xya.z());
      path.poses.push_back(pose);
    }

    if (publish && !path.poses.empty()) {
      geometry_msgs::msg::PoseArray poses;
      poses.header.frame_id = "map";
      poses.header.stamp = node_lifecycle_->get_clock()->now();
      for (auto & p : path.poses) {
        poses.poses.push_back(p.pose);
      }
      path_poses_pub_orig_->publish(poses);
      costmap_pub_->publishCostmap();
    }

    bool result = smoother_->smooth(path, rclcpp::Duration::from_seconds(10.0));

    if (publish && !path.poses.empty()) {
      geometry_msgs::msg::PoseArray poses;
      poses.header.frame_id = "map";
      poses.header.stamp = node_lifecycle_->get_clock()->now();
      for (auto & p : path.poses) {
        poses.poses.push_back(p.pose);
      }
      auto & pub = cmp ? path_poses_pub_cmp_ : path_poses_pub_;
      pub->publish(poses);
    }

    output.clear();
    output.reserve(path.poses.size());
    for (auto & pose : path.poses) {
      Eigen::Vector3d xya;
      xya.x() = pose.pose.position.x;
      xya.y() = pose.pose.position.y;
      tf2::Quaternion q;
      tf2::fromMsg(pose.pose.orientation, q);
      xya.z() = q.getAngle();
      output.push_back(xya);
    }
    return result;
  }

  typedef std::function<double (int i, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p,
      const Eigen::Vector3d & next_p)> QualityCriterion3;
  typedef std::function<double (int i, const Eigen::Vector3d & prev_p,
      const Eigen::Vector3d & p)> QualityCriterion2;
  typedef std::function<double (int i, const Eigen::Vector3d & p)> QualityCriterion1;
  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sqrt(sum(criterion(data[i])^2)/count(data))
   * @return Percentage of improvement (relative to input path quality)
   */
  double assessPathImprovement(
    const std::vector<Eigen::Vector3d> & input,
    const std::vector<Eigen::Vector3d> & output,
    const QualityCriterion3 & criterion,
    const QualityCriterion3 * criterion_out = nullptr)
  {
    if (!criterion_out) {
      criterion_out = &criterion;
    }
    double total_input_crit = 0.0;
    for (size_t i = 1; i < input.size() - 1; i++) {
      double input_crit = criterion(i, input[i - 1], input[i], input[i + 1]);
      total_input_crit += input_crit * input_crit;
    }
    total_input_crit = sqrt(total_input_crit / (input.size() - 2));

    double total_output_crit = 0.0;
    for (size_t i = 1; i < output.size() - 1; i++) {
      double output_crit = (*criterion_out)(i, output[i - 1], output[i], output[i + 1]);
      total_output_crit += output_crit * output_crit;
    }
    total_output_crit = sqrt(total_output_crit / (input.size() - 2));

    return (1.0 - total_output_crit / total_input_crit) * 100;
  }

  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sqrt(sum(criterion(data[i])^2)/count(data))
   * @return Percentage of improvement (relative to input path quality)
   */
  double assessPathImprovement(
    const std::vector<Eigen::Vector3d> & input,
    const std::vector<Eigen::Vector3d> & output,
    const QualityCriterion2 & criterion,
    const QualityCriterion2 * criterion_out = nullptr)
  {
    if (!criterion_out) {
      criterion_out = &criterion;
    }
    double total_input_crit = 0.0;
    for (size_t i = 1; i < input.size(); i++) {
      double input_crit = criterion(i, input[i - 1], input[i]);
      total_input_crit += input_crit * input_crit;
    }
    total_input_crit = sqrt(total_input_crit / (input.size() - 1));

    double total_output_crit = 0.0;
    for (size_t i = 1; i < output.size(); i++) {
      double output_crit = (*criterion_out)(i, output[i - 1], output[i]);
      total_output_crit += output_crit * output_crit;
    }
    total_output_crit = sqrt(total_output_crit / (output.size() - 1));

    return (1.0 - total_output_crit / total_input_crit) * 100;
  }

  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sqrt(sum(criterion(data[i])^2)/count(data))
   * @return Percentage of improvement (relative to input path quality)
   */
  double assessPathImprovement(
    const std::vector<Eigen::Vector3d> & input,
    const std::vector<Eigen::Vector3d> & output,
    const QualityCriterion1 & criterion,
    const QualityCriterion1 * criterion_out = nullptr)
  {
    if (!criterion_out) {
      criterion_out = &criterion;
    }
    double total_input_crit = 0.0;
    for (size_t i = 0; i < input.size(); i++) {
      double input_crit = criterion(i, input[i]);
      total_input_crit += input_crit * input_crit;
    }
    total_input_crit = sqrt(total_input_crit / input.size());

    double total_output_crit = 0.0;
    for (size_t i = 0; i < output.size(); i++) {
      double output_crit = (*criterion_out)(i, output[i]);
      total_output_crit += output_crit * output_crit;
    }
    total_output_crit = sqrt(total_output_crit / output.size());

    return (1.0 - total_output_crit / total_input_crit) * 100;
  }

  /**
   * @brief Worst pose improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = max(criterion(data[i]))
   * @return Percentage of improvement (relative to input path quality)
   */
  double assessWorstPoseImprovement(
    const std::vector<Eigen::Vector3d> & input,
    const std::vector<Eigen::Vector3d> & output,
    const QualityCriterion1 & criterion)
  {
    double max_input_crit = 0.0;
    for (size_t i = 0; i < input.size(); i++) {
      double input_crit = criterion(i, input[i]);
      max_input_crit = std::max(max_input_crit, input_crit);
    }

    double max_output_crit = 0.0;
    for (size_t i = 0; i < output.size(); i++) {
      double output_crit = criterion(i, output[i]);
      max_output_crit = std::max(max_output_crit, output_crit);
    }

    return (1.0 - max_output_crit / max_input_crit) * 100;
  }

  std::vector<Eigen::Vector3d> zigZaggedPath(
    const std::vector<Eigen::Vector3d> & input,
    double offset)
  {
    auto output = input;
    for (size_t i = 1; i < input.size() - 1; i++) {
      // add offset prependicular to path
      Eigen::Vector2d direction =
        (input[i + 1].block<2, 1>(0, 0) - input[i - 1].block<2, 1>(0, 0)).normalized();
      output[i].block<2, 1>(
        0,
        0) += Eigen::Vector2d(direction[1], -direction[0]) * offset * (i % 2 == 0 ? 1.0 : -1.0);
    }
    return output;
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<nav2_constrained_smoother::ConstrainedSmoother> smoother_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<DummyCostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
    path_poses_pub_orig_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr path_poses_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
    path_poses_pub_cmp_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;

  int cusp_i_ = -1;
  QualityCriterion3 mvmt_smoothness_criterion_ =
    [this](int i, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p,
      const Eigen::Vector3d & next_p) {
      Eigen::Vector2d prev_mvmt = p.block<2, 1>(0, 0) - prev_p.block<2, 1>(0, 0);
      Eigen::Vector2d next_mvmt = next_p.block<2, 1>(0, 0) - p.block<2, 1>(0, 0);
      if (i == cusp_i_) {
        next_mvmt = -next_mvmt;
      }
      return (next_mvmt - prev_mvmt).norm();
    };
};

TEST_F(SmootherTest, testingSmoothness)
{
  // set w_curve to 0.0 so that the whole job is upon w_smooth
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 0.0));
  reloadParams();

  std::vector<Eigen::Vector3d> sharp_turn_90 =
  {{0, 0, 0},
    {0.1, 0, 0},
    {0.2, 0, 0},
    {0.3, 0, M_PI / 4},
    {0.3, 0.1, M_PI / 2},
    {0.3, 0.2, M_PI / 2},
    {0.3, 0.3, M_PI / 2}
  };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  double mvmt_smoothness_improvement =
    assessPathImprovement(sharp_turn_90, smoothed_path, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 55.3, 1.0);

  auto orientation_smoothness_criterion =
    [](int, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p) {
      return angles::normalize_angle(p.z() - prev_p.z());
    };
  double orientation_smoothness_improvement =
    assessPathImprovement(sharp_turn_90, smoothed_path, orientation_smoothness_criterion);
  EXPECT_GT(orientation_smoothness_improvement, 0.0);
  EXPECT_NEAR(orientation_smoothness_improvement, 38.7, 1.0);

  // path with a cusp
  std::vector<Eigen::Vector3d> sharp_turn_90_then_reverse =
  {{0, 0, 0},
    {0.1, 0, 0},
    {0.2, 0, 0},
    {0.3, 0, 0},
    {0.4, 0, 0},
    {0.5, 0, 0},
    {0.6, 0, M_PI / 4},
    {0.6, -0.1, M_PI / 2},
    {0.6, -0.2, M_PI / 2},
    {0.6, -0.3, M_PI / 2},
    {0.6, -0.4, M_PI / 2},
    {0.6, -0.5, M_PI / 2},
    {0.6, -0.6, M_PI / 2}
  };
  cusp_i_ = 6;

  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path));

  mvmt_smoothness_improvement =
    assessPathImprovement(sharp_turn_90_then_reverse, smoothed_path, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 37.2, 1.0);

  orientation_smoothness_improvement =
    assessPathImprovement(
    sharp_turn_90_then_reverse, smoothed_path,
    orientation_smoothness_criterion);
  EXPECT_GT(orientation_smoothness_improvement, 0.0);
  EXPECT_NEAR(orientation_smoothness_improvement, 28.5, 1.0);

  SUCCEED();
}

TEST_F(SmootherTest, testingAnchoringToOriginalPath)
{
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 30.0));
  // set w_curve to 0.0, we don't care about turning radius in this test...
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 0.0));
  // first keep w_dist at 0.0 to generate an unanchored smoothed path
  reloadParams();

  std::vector<Eigen::Vector3d> sharp_turn_90 =
  {{0, 0, 0},
    {0.1, 0, 0},
    {0.2, 0, 0},
    {0.3, 0, M_PI / 4},
    {0.3, 0.1, M_PI / 2},
    {0.3, 0.2, M_PI / 2},
    {0.3, 0.3, M_PI / 2}
  };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  // then update w_dist and compare the results
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 30.0));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_anchored;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path_anchored));

  auto origin_similarity_criterion =
    [&sharp_turn_90](int i, const Eigen::Vector3d & p) {
      return (p.block<2, 1>(0, 0) - sharp_turn_90[i].block<2, 1>(0, 0)).norm();
    };
  double origin_similarity_improvement =
    assessPathImprovement(smoothed_path, smoothed_path_anchored, origin_similarity_criterion);
  EXPECT_GT(origin_similarity_improvement, 0.0);
  EXPECT_NEAR(origin_similarity_improvement, 45.5, 1.0);

  SUCCEED();
}

TEST_F(SmootherTest, testingMaxCurvature)
{
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 30.0));
  // set w_smooth to a small value so that the whole job is upon w_curve
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 0.3));
  // let's give the smoother more time since w_smooth is so small
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.max_iterations", 500));
  reloadParams();

  // smoother should increase radius from infeasible 0.3 to feasible 0.4
  std::vector<Eigen::Vector3d> radius_0_3_turn_90 =
  {{0, 0, 0},
    {0.1, 0, 0},
    {0.2, 0, 0},
    {0.2 + 0.3 * sin(M_PI / 12), 0.3 * (1 - cos(M_PI / 12)), 0},
    {0.2 + 0.3 * sin(M_PI * 2 / 12), 0.3 * (1 - cos(M_PI * 2 / 12)), 0},
    {0.2 + 0.3 * sin(M_PI * 3 / 12), 0.3 * (1 - cos(M_PI * 3 / 12)), 0},
    {0.2 + 0.3 * sin(M_PI * 4 / 12), 0.3 * (1 - cos(M_PI * 4 / 12)), 0},
    {0.2 + 0.3 * sin(M_PI * 5 / 12), 0.3 * (1 - cos(M_PI * 5 / 12)), 0},
    {0.5, 0.3, M_PI / 2},
    {0.5, 0.4, M_PI / 2},
    {0.5, 0.5, M_PI / 2}
  };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(radius_0_3_turn_90, smoothed_path));

  // we don't expect result to be smoother than original as w_smooth is too low
  // but let's check for large discontinuities using a well chosen upper bound
  auto upper_bound = zigZaggedPath(radius_0_3_turn_90, 0.01);
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);

  // smoothed path points should form a circle with radius 0.4
  for (size_t i = 1; i < smoothed_path.size() - 1; i++) {
    auto & p = smoothed_path[i];
    double r = (p.block<2, 1>(0, 0) - Eigen::Vector2d(0.1, 0.4)).norm();
    EXPECT_NEAR(r, 0.4, 0.01);
  }

  // path with a cusp
  // smoother should increase radius from infeasible 0.3 to feasible 0.4
  std::vector<Eigen::Vector3d> radius_0_3_turn_90_then_reverse_turn_90 =
  {{0, 0, 0},
    {0.1, 0, 0},
    {0.2, 0, 0},
    {0.2 + 0.3 * sin(M_PI / 12), 0.3 * (1 - cos(M_PI / 12)), M_PI / 12},
    {0.2 + 0.3 * sin(M_PI * 2 / 12), 0.3 * (1 - cos(M_PI * 2 / 12)), M_PI *2 / 12},
    {0.2 + 0.3 * sin(M_PI * 3 / 12), 0.3 * (1 - cos(M_PI * 3 / 12)), M_PI *3 / 12},
    {0.2 + 0.3 * sin(M_PI * 4 / 12), 0.3 * (1 - cos(M_PI * 4 / 12)), M_PI *4 / 12},
    {0.2 + 0.3 * sin(M_PI * 5 / 12), 0.3 * (1 - cos(M_PI * 5 / 12)), M_PI *5 / 12},
    {0.5, 0.3, M_PI / 2},
    {0.8 - 0.3 * sin(M_PI * 5 / 12), 0.3 * (1 - cos(M_PI * 5 / 12)), M_PI *7 / 12},
    {0.8 - 0.3 * sin(M_PI * 4 / 12), 0.3 * (1 - cos(M_PI * 4 / 12)), M_PI *8 / 12},
    {0.8 - 0.3 * sin(M_PI * 3 / 12), 0.3 * (1 - cos(M_PI * 3 / 12)), M_PI *9 / 12},
    {0.8 - 0.3 * sin(M_PI * 2 / 12), 0.3 * (1 - cos(M_PI * 2 / 12)), M_PI *10 / 12},
    {0.8 - 0.3 * sin(M_PI / 12), 0.3 * (1 - cos(M_PI / 12)), M_PI *11 / 12},
    {0.8, 0, M_PI},
    {0.9, 0, M_PI},
    {1.0, 0, M_PI}
  };

  EXPECT_TRUE(smoothPath(radius_0_3_turn_90_then_reverse_turn_90, smoothed_path));

  // we don't expect result to be smoother than original as w_smooth is too low
  // but let's check for large discontinuities using a well chosen upper bound
  cusp_i_ = 8;
  upper_bound = zigZaggedPath(radius_0_3_turn_90_then_reverse_turn_90, 0.01);
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);

  // smoothed path points should form a circle with radius 0.4
  // for both forward and reverse movements
  for (size_t i = 1; i < smoothed_path.size() - 1; i++) {
    auto & p = smoothed_path[i];
    double r = (p.block<2, 1>(0, 0) - Eigen::Vector2d(i <= 8 ? 0.1 : 0.9, 0.4)).norm();
    EXPECT_NEAR(r, 0.4, 0.01);
  }

  SUCCEED();
}

TEST_F(SmootherTest, testingObstacleAvoidance)
{
  auto costmap = costmap_sub_->getCostmap();
  nav2_costmap_2d::FootprintCollisionChecker collision_checker(costmap);
  nav2_costmap_2d::Footprint footprint;

  auto cost_avoidance_criterion =
    [&collision_checker, &footprint](int, const Eigen::Vector3d & p) {
      return collision_checker.footprintCostAtPose(p[0], p[1], p[2], footprint);
    };

  // a symmetric footprint (diff-drive with 4 actuated wheels)
  footprint.push_back(pointMsg(0.4, 0.25));
  footprint.push_back(pointMsg(-0.4, 0.25));
  footprint.push_back(pointMsg(-0.4, -0.25));
  footprint.push_back(pointMsg(0.4, -0.25));

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 2000000.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.015));
  reloadParams();

  std::vector<Eigen::Vector3d> straight_near_obstacle =
  {{0.05, 0.05, 0},
    {0.45, 0.05, 0},
    {0.85, 0.05, 0},
    {1.25, 0.05, 0},
    {1.65, 0.05, 0},
    {2.05, 0.05, 0},
    {2.45, 0.05, 0},
    {2.85, 0.05, 0},
    {3.25, 0.05, 0},
    {3.65, 0.05, 0},
    {4.05, 0.05, 0}
  };

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(straight_near_obstacle, smoothed_path));

  // we don't expect result to be smoother than original as original straight line was 100% smooth
  // but let's check for large discontinuities using a well chosen upper bound
  auto upper_bound = zigZaggedPath(straight_near_obstacle, 0.01);
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);

  double cost_avoidance_improvement = assessPathImprovement(
    straight_near_obstacle, smoothed_path,
    cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement, 9.4, 1.0);
}

TEST_F(SmootherTest, testingObstacleAvoidanceNearCusps)
{
  auto costmap = costmap_sub_->getCostmap();
  nav2_costmap_2d::FootprintCollisionChecker collision_checker(costmap);
  nav2_costmap_2d::Footprint footprint;

  auto cost_avoidance_criterion =
    [&collision_checker, &footprint](int, const Eigen::Vector3d & p) {
      return collision_checker.footprintCostAtPose(p[0], p[1], p[2], footprint);
    };

  // path with a cusp
  std::vector<Eigen::Vector3d> cusp_near_obstacle =
  {{0.05, 0.05, 0},
    {0.15, 0.05, 0},
    {0.25, 0.05, 0},
    {0.35, 0.05, 0},
    {0.45, 0.05, 0},
    {0.55, 0.05, 0},
    {0.65, 0.05, 0},
    {0.75, 0.05, 0},
    {0.85, 0.05, 0},
    {0.95, 0.05, 0},
    {1.05, 0.05, 0},
    {1.15, 0.05, 0},
    {1.25, 0.05, 0},
    {1.25 + 0.4 * sin(M_PI / 12), 0.4 * (1 - cos(M_PI / 12)) + 0.05, M_PI / 12},
    {1.25 + 0.4 * sin(M_PI * 2 / 12), 0.4 * (1 - cos(M_PI * 2 / 12)) + 0.05, M_PI *2 / 12},
    {1.25 + 0.4 * sin(M_PI * 3 / 12), 0.4 * (1 - cos(M_PI * 3 / 12)) + 0.05, M_PI *3 / 12},
    {1.25 + 0.4 * sin(M_PI * 4 / 12), 0.4 * (1 - cos(M_PI * 4 / 12)) + 0.05, M_PI *4 / 12},
    {1.25 + 0.4 * sin(M_PI * 5 / 12), 0.4 * (1 - cos(M_PI * 5 / 12)) + 0.05, M_PI *5 / 12},
    {1.65, 0.45, M_PI / 2},
    {2.05 - 0.4 * sin(M_PI * 5 / 12), 0.4 * (1 - cos(M_PI * 5 / 12)) + 0.05, M_PI *7 / 12},
    {2.05 - 0.4 * sin(M_PI * 4 / 12), 0.4 * (1 - cos(M_PI * 4 / 12)) + 0.05, M_PI *8 / 12},
    {2.05 - 0.4 * sin(M_PI * 3 / 12), 0.4 * (1 - cos(M_PI * 3 / 12)) + 0.05, M_PI *9 / 12},
    {2.05 - 0.4 * sin(M_PI * 2 / 12), 0.4 * (1 - cos(M_PI * 2 / 12)) + 0.05, M_PI *10 / 12},
    {2.05 - 0.4 * sin(M_PI / 12), 0.4 * (1 - cos(M_PI / 12)) + 0.05, M_PI *11 / 12},
    {2.05, 0.05, M_PI},
    {2.15, 0.05, M_PI},
    {2.25, 0.05, M_PI},
    {2.35, 0.05, M_PI},
    {2.45, 0.05, M_PI},
    {2.55, 0.05, M_PI},
    {2.65, 0.05, M_PI},
    {2.75, 0.05, M_PI},
    {2.85, 0.05, M_PI},
    {2.95, 0.05, M_PI},
    {3.05, 0.05, M_PI},
    {3.15, 0.05, M_PI},
    {3.25, 0.05, M_PI},
    {3.35, 0.05, M_PI},
    {3.45, 0.05, M_PI},
    {3.55, 0.05, M_PI},
    {3.65, 0.05, M_PI},
    {3.75, 0.05, M_PI},
    {3.85, 0.05, M_PI},
    {3.95, 0.05, M_PI},
    {4.05, 0.05, M_PI}
  };
  cusp_i_ = 18;

  // we don't expect result to be smoother than original
  // but let's check for large discontinuities using a well chosen upper bound
  auto upper_bound = zigZaggedPath(cusp_near_obstacle, 0.01);

  /////////////////////////////////////////////////////
  // testing option to pay extra attention near cusps

  // extra attention near cusps option is more significant with a long footprint
  footprint.clear();
  footprint.push_back(pointMsg(0.4, 0.2));
  footprint.push_back(pointMsg(-0.4, 0.2));
  footprint.push_back(pointMsg(-0.4, -0.2));
  footprint.push_back(pointMsg(0.4, -0.2));

  // first smooth with homogeneous w_cost to compare
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 15000.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.015));
  // higher w_curve significantly decreases convergence speed here
  // path feasibility can be restored by subsequent resmoothing with higher w_curve
  // TODO(afrixs): tune ceres optimizer to "converge" faster,
  //               see http://ceres-solver.org/nnls_solving.html
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 1.0));
  // let's have more iterations so that the improvement is more significant
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.max_iterations", 500));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path, true, true));
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path, mvmt_smoothness_criterion_), 0.0);
  double cost_avoidance_improvement_simple = assessPathImprovement(
    cusp_near_obstacle,
    smoothed_path,
    cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_simple, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement_simple, 42.6, 1.0);
  double worst_cost_improvement_simple = assessWorstPoseImprovement(
    cusp_near_obstacle,
    smoothed_path,
    cost_avoidance_criterion);
  RCLCPP_INFO(
    rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp, simple): %lf, %lf",
    cost_avoidance_improvement_simple, worst_cost_improvement_simple);
  EXPECT_GE(worst_cost_improvement_simple, 0.0);


  // then update parameters so that robot is not so afraid of obstacles
  // during simple movement but pays extra attention during rotations near cusps
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0052));
  node_lifecycle_->set_parameter(
    rclcpp::Parameter("SmoothPath.w_cost_cusp_multiplier", 0.027 / 0.0052));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", 2.5));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.fn_tol", 1e-15));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_ecc;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_ecc, true, false));
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path_ecc, mvmt_smoothness_criterion_), 0.0);
  double cost_avoidance_improvement_extra_careful_cusp = assessPathImprovement(
    cusp_near_obstacle,
    smoothed_path_ecc,
    cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_extra_careful_cusp, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement_extra_careful_cusp, 44.2, 1.0);
  double worst_cost_improvement_extra_careful_cusp = assessWorstPoseImprovement(
    cusp_near_obstacle,
    smoothed_path_ecc,
    cost_avoidance_criterion);
  RCLCPP_INFO(
    rclcpp::get_logger("ceres_smoother"), "Cost avoidance improvement (cusp, ecc): %lf, %lf",
    cost_avoidance_improvement_extra_careful_cusp, worst_cost_improvement_extra_careful_cusp);
  EXPECT_GE(worst_cost_improvement_extra_careful_cusp, 0.0);
  EXPECT_GE(worst_cost_improvement_extra_careful_cusp, worst_cost_improvement_simple);
  EXPECT_GT(cost_avoidance_improvement_extra_careful_cusp, cost_avoidance_improvement_simple);

  // although extra careful cusp optimization avoids cost better than simple one,
  // overall the path doesn't need to deflect so much from original, since w_cost is smaller
  // and thus the obstacles are avoided mostly in dangerous zones around cusps
  auto origin_similarity_criterion =
    [&cusp_near_obstacle](int i, const Eigen::Vector3d & p) {
      return (p.block<2, 1>(0, 0) - cusp_near_obstacle[i].block<2, 1>(0, 0)).norm();
    };
  double origin_similarity_improvement =
    assessPathImprovement(smoothed_path, smoothed_path_ecc, origin_similarity_criterion);
  RCLCPP_INFO(
    rclcpp::get_logger(
      "ceres_smoother"), "Original similarity improvement (cusp, ecc vs. simple): %lf",
    origin_similarity_improvement);
  EXPECT_GT(origin_similarity_improvement, 0.0);
  EXPECT_NEAR(origin_similarity_improvement, 0.43, 0.02);


  /////////////////////////////////////////////////////
  // testing asymmetric footprint options

  // (diff-drive with 2 actuated wheels and 2 caster wheels)
  footprint.clear();
  footprint.push_back(pointMsg(0.15, 0.2));
  footprint.push_back(pointMsg(-0.65, 0.2));
  footprint.push_back(pointMsg(-0.65, -0.2));
  footprint.push_back(pointMsg(0.15, -0.2));

  // reset parameters back to homogeneous and shift cost check point to the center of the footprint
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 15000.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 1.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.015));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", -1.0));
  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.cost_check_points",
      std::vector<double>({-0.05, 0.0, 0.5, -0.45, 0.0, 0.5})  // x1, y1, weight1, x2, y2, weight2
  ));
  reloadParams();

  // cost improvement is different for path smoothed by original optimizer
  // since the footprint has changed
  cost_avoidance_improvement_simple = assessPathImprovement(
    cusp_near_obstacle, smoothed_path,
    cost_avoidance_criterion);
  worst_cost_improvement_simple = assessWorstPoseImprovement(
    cusp_near_obstacle, smoothed_path,
    cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_simple, 0.0);
  RCLCPP_INFO(
    rclcpp::get_logger(
      "ceres_smoother"), "Cost avoidance improvement (cusp_shifted, simple): %lf, %lf",
    cost_avoidance_improvement_simple, worst_cost_improvement_simple);
  EXPECT_NEAR(cost_avoidance_improvement_simple, 40.2, 1.0);

  // now smooth using the new optimizer with cost check point shifted
  std::vector<Eigen::Vector3d> smoothed_path_scc;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_scc));
  EXPECT_GT(assessPathImprovement(upper_bound, smoothed_path_scc, mvmt_smoothness_criterion_), 0.0);
  double cost_avoidance_improvement_shifted_cost_check = assessPathImprovement(
    cusp_near_obstacle,
    smoothed_path_scc,
    cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_shifted_cost_check, 0.0);
  EXPECT_NEAR(cost_avoidance_improvement_shifted_cost_check, 42.0, 1.0);
  double worst_cost_improvement_shifted_cost_check = assessWorstPoseImprovement(
    cusp_near_obstacle,
    smoothed_path_scc,
    cost_avoidance_criterion);
  RCLCPP_INFO(
    rclcpp::get_logger(
      "ceres_smoother"), "Cost avoidance improvement (cusp_shifted, scc): %lf, %lf",
    cost_avoidance_improvement_shifted_cost_check, worst_cost_improvement_shifted_cost_check);
  EXPECT_GE(worst_cost_improvement_shifted_cost_check, 0.0);
  EXPECT_GE(worst_cost_improvement_shifted_cost_check, worst_cost_improvement_simple);
  EXPECT_GT(cost_avoidance_improvement_shifted_cost_check, cost_avoidance_improvement_simple);

  // same results should be achieved with unnormalized weights
  // (testing automatic weights normalization, i.e. using avg instead of sum)
  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.cost_check_points",
      std::vector<double>({-0.05, 0.0, 1.0, -0.45, 0.0, 1.0})  // x1, y1, weight1, x2, y2, weight2
  ));
  reloadParams();
  std::vector<Eigen::Vector3d> smoothed_path_scc_unnormalized;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_scc_unnormalized));
  EXPECT_EQ(smoothed_path_scc, smoothed_path_scc_unnormalized);

  ////////////////////////////////////////
  // compare also with extra careful cusp

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0052));
  node_lifecycle_->set_parameter(
    rclcpp::Parameter("SmoothPath.w_cost_cusp_multiplier", 0.027 / 0.0052));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.cusp_zone_length", 2.5));
  // we need much more iterations here since it's a more complicated problem
  // TODO(afrixs): tune ceres optimizer to "converge" faster
  //               see http://ceres-solver.org/nnls_solving.html
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.optimizer.max_iterations", 1500));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_scce;
  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_scce));
  EXPECT_GT(
    assessPathImprovement(upper_bound, smoothed_path_scce, mvmt_smoothness_criterion_),
    0.0);
  double cost_avoidance_improvement_shifted_extra = assessPathImprovement(
    cusp_near_obstacle,
    smoothed_path_scce,
    cost_avoidance_criterion);
  double worst_cost_improvement_shifted_extra = assessWorstPoseImprovement(
    cusp_near_obstacle,
    smoothed_path_scce,
    cost_avoidance_criterion);
  RCLCPP_INFO(
    rclcpp::get_logger(
      "ceres_smoother"), "Cost avoidance improvement (cusp_shifted, scce): %lf, %lf",
    cost_avoidance_improvement_shifted_extra, worst_cost_improvement_shifted_extra);
  EXPECT_NEAR(cost_avoidance_improvement_shifted_extra, 51.0, 1.0);
  EXPECT_GE(worst_cost_improvement_shifted_extra, 0.0);

  // resmooth extra careful cusp with same conditions (higher max_iterations)
  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.cost_check_points",
      std::vector<double>()));
  reloadParams();

  EXPECT_TRUE(smoothPath(cusp_near_obstacle, smoothed_path_ecc));
  cost_avoidance_improvement_extra_careful_cusp = assessPathImprovement(
    cusp_near_obstacle,
    smoothed_path_ecc,
    cost_avoidance_criterion);
  worst_cost_improvement_extra_careful_cusp = assessWorstPoseImprovement(
    cusp_near_obstacle,
    smoothed_path_ecc,
    cost_avoidance_criterion);
  EXPECT_GT(cost_avoidance_improvement_extra_careful_cusp, 0.0);
  RCLCPP_INFO(
    rclcpp::get_logger(
      "ceres_smoother"), "Cost avoidance improvement (cusp_shifted, ecc): %lf, %lf",
    cost_avoidance_improvement_extra_careful_cusp, worst_cost_improvement_extra_careful_cusp);
  EXPECT_NEAR(cost_avoidance_improvement_extra_careful_cusp, 48.5, 1.0);
  EXPECT_GT(
    cost_avoidance_improvement_shifted_extra,
    cost_avoidance_improvement_extra_careful_cusp);
  // worst cost improvement is a bit lower but only by 5% so it's not a big deal
  EXPECT_GE(worst_cost_improvement_shifted_extra, worst_cost_improvement_extra_careful_cusp - 6.0);

  SUCCEED();
}

TEST_F(SmootherTest, testingDownsamplingUpsampling)
{
  // path with a cusp
  std::vector<Eigen::Vector3d> sharp_turn_90_then_reverse =
  {{0, 0, 0},
    {0.1, 0, 0},
    {0.2, 0, 0},
    {0.3, 0, 0},
    {0.4, 0, 0},
    {0.5, 0, 0},
    {0.6, 0, M_PI / 4},
    {0.6, -0.1, M_PI / 2},
    {0.6, -0.2, M_PI / 2},
    {0.6, -0.3, M_PI / 2},
    {0.6, -0.4, M_PI / 2},
    {0.6, -0.5, M_PI / 2},
    {0.6, -0.6, M_PI / 2}
  };
  cusp_i_ = 6;

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_downsampling_factor", 2));
  // downsample only
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.reversing_enabled", false));
  reloadParams();
  std::vector<Eigen::Vector3d> smoothed_path_downsampled;
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path_downsampled));
  // first two, last two and every 2nd pose between them
  EXPECT_EQ(smoothed_path_downsampled.size(), 8u);

  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.reversing_enabled", true));
  reloadParams();
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path_downsampled));
  // same but downsampling is reset on cusp
  EXPECT_EQ(smoothed_path_downsampled.size(), 9u);

  // upsample to original size
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 1));
  reloadParams();
  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path));
  EXPECT_EQ(smoothed_path.size(), sharp_turn_90_then_reverse.size());

  cusp_i_ = 4;  // for downsampled path
  int cusp_i_out = 6;  // for upsampled path
  QualityCriterion3 mvmt_smoothness_criterion_out =
    [&cusp_i_out](int i, const Eigen::Vector3d & prev_p, const Eigen::Vector3d & p,
      const Eigen::Vector3d & next_p) {
      Eigen::Vector2d prev_mvmt = p.block<2, 1>(0, 0) - prev_p.block<2, 1>(0, 0);
      Eigen::Vector2d next_mvmt = next_p.block<2, 1>(0, 0) - p.block<2, 1>(0, 0);
      if (i == cusp_i_out) {
        next_mvmt = -next_mvmt;
      }
      return (next_mvmt - prev_mvmt).norm();
    };

  double smoothness_improvement = assessPathImprovement(
    smoothed_path_downsampled, smoothed_path,
    mvmt_smoothness_criterion_,
    &mvmt_smoothness_criterion_out);
  // more poses -> smoother path
  EXPECT_GT(smoothness_improvement, 0.0);
  EXPECT_NEAR(smoothness_improvement, 63.9, 1.0);

  // upsample above original size
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.path_upsampling_factor", 2));
  reloadParams();
  EXPECT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path));
  // every pose except last produces 2 poses
  EXPECT_EQ(smoothed_path.size(), sharp_turn_90_then_reverse.size() * 2 - 1);
  cusp_i_out = 12;  // for upsampled path
  smoothness_improvement = assessPathImprovement(
    smoothed_path_downsampled, smoothed_path,
    mvmt_smoothness_criterion_,
    &mvmt_smoothness_criterion_out);
  // even more poses -> even smoother path
  EXPECT_GT(smoothness_improvement, 0.0);
  EXPECT_NEAR(smoothness_improvement, 82.2, 1.0);
}

TEST_F(SmootherTest, testingStartGoalOrientations)
{
  std::vector<Eigen::Vector3d> sharp_turn_90 =
  {{0, 0, 0},
    {0.1, 0, 0},
    {0.2, 0, 0},
    {0.3, 0, M_PI / 4},
    {0.3, 0.1, M_PI / 2},
    {0.3, 0.2, M_PI / 2},
    {0.3, 0.3, M_PI / 2}
  };

  // Keep start and goal orientations (by default)
  std::vector<Eigen::Vector3d> smoothed_path;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  double mvmt_smoothness_improvement =
    assessPathImprovement(sharp_turn_90, smoothed_path, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 55.2, 1.0);
  // no change in orientations
  EXPECT_NEAR(smoothed_path.front()[2], 0, 0.001);
  EXPECT_NEAR(smoothed_path.back()[2], M_PI / 2, 0.001);

  // Overwrite start/goal orientations
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_start_orientation", false));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.keep_goal_orientation", false));
  reloadParams();

  sharp_turn_90[0][2] = M_PI;  // forward/reverse of start pose should not matter
  std::vector<Eigen::Vector3d> smoothed_path_sg_overwritten;
  EXPECT_TRUE(smoothPath(sharp_turn_90, smoothed_path_sg_overwritten));

  mvmt_smoothness_improvement =
    assessPathImprovement(smoothed_path, smoothed_path_sg_overwritten, mvmt_smoothness_criterion_);
  EXPECT_GT(mvmt_smoothness_improvement, 0.0);
  EXPECT_NEAR(mvmt_smoothness_improvement, 58.9, 1.0);
  // orientations adjusted to follow the path
  EXPECT_NEAR(smoothed_path_sg_overwritten.front()[2], M_PI / 8, 0.1);
  EXPECT_NEAR(smoothed_path_sg_overwritten.back()[2], 3 * M_PI / 8, 0.1);

  // test short paths
  std::vector<Eigen::Vector3d> short_screwed_path =
  {{0, 0, M_PI * 0.25},
    {0.1, 0, -M_PI * 0.25}
  };

  std::vector<Eigen::Vector3d> adjusted_path;
  EXPECT_TRUE(smoothPath(short_screwed_path, adjusted_path));
  EXPECT_NEAR(adjusted_path.front()[2], 0, 0.001);
  EXPECT_NEAR(adjusted_path.back()[2], 0, 0.001);

  short_screwed_path[0][2] = M_PI * 0.75;  // start is stronger than goal
  EXPECT_TRUE(smoothPath(short_screwed_path, adjusted_path));
  EXPECT_NEAR(adjusted_path.front()[2], M_PI, 0.001);
  EXPECT_NEAR(adjusted_path.back()[2], M_PI, 0.001);

  std::vector<Eigen::Vector3d> one_pose_path = {{0, 0, M_PI * 0.75}};
  EXPECT_TRUE(smoothPath(one_pose_path, adjusted_path));
  EXPECT_NEAR(adjusted_path.front()[2], M_PI * 0.75, 0.001);
}

TEST_F(SmootherTest, testingCostCheckPointsParamValidity)
{
  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.cost_check_points",
      std::vector<double>()));
  reloadParams();

  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.cost_check_points",
      std::vector<double>({0, 0, 0, 0, 0, 0, 0, 0, 0})));  // multiple of 3 is ok
  reloadParams();

  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.cost_check_points",
      std::vector<double>({0, 0})));
  EXPECT_THROW(reloadParams(), std::runtime_error);
}

TEST_F(SmootherTest, testingLinearSolverTypeParamValidity)
{
  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.optimizer.linear_solver_type",
      "SPARSE_NORMAL_CHOLESKY"));
  reloadParams();

  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.optimizer.linear_solver_type",
      "DENSE_QR"));
  reloadParams();

  node_lifecycle_->set_parameter(
    rclcpp::Parameter(
      "SmoothPath.optimizer.linear_solver_type",
      "INVALID_SOLVER"));
  EXPECT_THROW(reloadParams(), std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
