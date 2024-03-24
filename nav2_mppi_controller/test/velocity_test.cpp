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

#include <iostream>
#include <fstream>
#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_mppi_controller/optimizer.hpp>
#include "utils/utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

// Tests basic transition from configure->active->process->deactive->cleanup

TEST(VelocityTest, ParameterSweep)
{
  // Costmap Settings
  double resolution = 0.1;
  unsigned int cells_x = 30 / resolution;
  unsigned int cells_y = 30 / resolution;
  double origin_x = 0.0;
  double origin_y = 0.0;
  unsigned char cost_map_default_value = 0;
  double footprint_size = 2.0;  // Overridden with setRobotFootprint

  // ControllerSettings
  double controller_frequency = 20.0;
  bool visualize = true;

  TestCostmapSettings costmap_settings{cells_x, cells_y, origin_x, origin_y,
    resolution, cost_map_default_value, footprint_size};

  TestControllerSettings controller_settings{controller_frequency, visualize};

  // OptimizerSettings
  TestOptimizerSettings optimizer_settings{};
  optimizer_settings.model_dt = 0.05;
  optimizer_settings.batch_size = 1000;
  optimizer_settings.time_steps = 60;
  optimizer_settings.iteration_count = 1;
  optimizer_settings.motion_model = "Ackermann";
  optimizer_settings.consider_footprint = true;
  optimizer_settings.vx_max = 0.5;
  optimizer_settings.vx_min = -0.35;
  optimizer_settings.vy_max = 0.5;
  optimizer_settings.wz_max = 1.9;
  optimizer_settings.vx_std = 0.2;
  optimizer_settings.vy_std = 0.2;
  optimizer_settings.wz_std = 0.4;

  std::vector<std::string> critics({
    {"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}});

  // Limit critics being used
  // critics.clear();
  // critics.push_back("PathFollowCritic");
  // critics.push_back("PreferForwardCritic");

  // Explore parameters for the Polaris ATV Ackermann
  optimizer_settings.vx_max = 3.0;
  optimizer_settings.vx_min = -1.0;
  optimizer_settings.wz_max = 0.52;
  optimizer_settings.wz_std = 0.1;

  // Node Options
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  setUpControllerParams(controller_settings, params);
  setUpOptimizerParams(optimizer_settings, critics, params);
  options.parameter_overrides(params);

  auto node = getDummyNode(options);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  costmap_ros->setRobotFootprint(getDummyRectangleFootprint(3.6, 1.8, 1.0, 0.0));

  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());
  parameters_handler->start();

  TestPose start_pose = costmap_settings.getCenterPose();
  // Go straight in x direction.
  unsigned int path_points = 50u;
  double path_step_x = costmap_settings.resolution;
  double path_step_y = 0;
  TestPathSettings path_settings{start_pose, path_points, path_step_x, path_step_y};

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto path = getIncrementalDummyPath(node, path_settings);

  nav2_core::GoalChecker * dummy_goal_checker{nullptr};

  auto feedbackFuncs = std::tuple(
    [](const geometry_msgs::msg::TwistStamped & cmd_vel, geometry_msgs::msg::Twist & velocity) {
      velocity.linear = cmd_vel.twist.linear;
      velocity.angular = cmd_vel.twist.angular;
    },
    [](const geometry_msgs::msg::TwistStamped & cmd_vel, geometry_msgs::msg::Twist & velocity) {
      velocity.linear.x = 0.95 * velocity.linear.x + 0.05 * cmd_vel.twist.linear.x;
      velocity.angular.z = 0.95 * velocity.angular.z + 0.05 * cmd_vel.twist.angular.z;
    },
    [](const geometry_msgs::msg::TwistStamped & cmd_vel, geometry_msgs::msg::Twist & velocity) {
      // Just drive straight.
      velocity.linear = cmd_vel.twist.linear;
      velocity.angular.z = 0;
    },
    [](const geometry_msgs::msg::TwistStamped & cmd_vel, geometry_msgs::msg::Twist & velocity) {
      // Just drive straight.
      velocity.linear.x = 0.95 * velocity.linear.x + 0.05 * cmd_vel.twist.linear.x;
      velocity.angular.z = 0.0;
    });

  auto v_in = getDummyTwist();
  auto cmd_vel = getDummyTwistStamped();
  auto feedbackFunc = std::get<2>(feedbackFuncs);

  int i;
  int j;
  int k;
  int vx_std_iter_start = 2;
  int vx_std_iter_end = 7;
  double vx_std_iter_delta = 0.1;
  int vx_iter_start = 1;
  int vx_iter_end = 10;
  int vx_iter_delta = 0.05;
  int max_evalControl_iter = 4.0 / optimizer_settings.model_dt;  // Run for 4 seconds

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  std::ofstream fresults;
  std::string fn("nav2_mppi_controller_velocity_test.csv");
  fresults.open(fn, std::ios::out | std::ios::trunc);

  for (k = vx_std_iter_start; k <= vx_std_iter_end; k++) {
    double vx_std = vx_std_iter_delta * k;
    auto results = rec_param->set_parameters_atomically(
      {rclcpp::Parameter("dummy.verbose", true),
        rclcpp::Parameter("dummy.vx_std", vx_std),
        rclcpp::Parameter("dummy.regenerate_noises", false),
        rclcpp::Parameter("dummy.dump_noises", true),
        rclcpp::Parameter("dummy.noise_seed", 1337),
        rclcpp::Parameter("AckermannConstraints.min_turning_r", 2.75),
        rclcpp::Parameter("dummy.PathFollowCritic.cost_weight", 5.0)});

    rclcpp::spin_until_future_complete(
      node->get_node_base_interface(),
      results);

    EXPECT_EQ(node->get_parameter("dummy.vx_std").as_double(), vx_std);

    // Simulate response to generated cmd_vel by running through feedbackFunc
    // by varying the starting velocity
    for (j = vx_iter_start; j <= vx_iter_end; j++) {
      // NOTE: If regenerate_noises == true we'd need to wait
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));

      v_in = getDummyTwist();
      EXPECT_NEAR(v_in.linear.x, 0.0, 1e-6);
      EXPECT_NEAR(v_in.angular.z, 0.0, 1e-6);
      v_in.linear.x = vx_iter_delta * j;
      v_in.angular.z = 0.0;

      cmd_vel = getDummyTwistStamped();
      EXPECT_NEAR(cmd_vel.twist.linear.x, 0.0, 1e-6);
      EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 1e-6);

      optimizer->reset();
      EXPECT_EQ(node->get_parameter("dummy.vx_std").as_double(), vx_std);
      fresults << "#k,j,i,vx_max,vx_std,wz_max,wz_std,"
               << "vx_in,wz_in,cmd_vel_vx,cmd_vel_wz" << std::endl;
      for (i = 0; i < max_evalControl_iter; i++) {
        cmd_vel = optimizer->evalControl(pose, v_in, path, dummy_goal_checker);

        fresults << k
                 << "," << j
                 << "," << i
                 << "," << optimizer_settings.vx_max
                 << "," << vx_std
                 << "," << optimizer_settings.wz_max
                 << "," << optimizer_settings.wz_std
                 << "," << v_in.linear.x
                 << "," << v_in.angular.z
                 << "," << cmd_vel.twist.linear.x
                 << "," << cmd_vel.twist.angular.z
                 << std::endl;

        feedbackFunc(cmd_vel, v_in);
      }
    }
  }
}
