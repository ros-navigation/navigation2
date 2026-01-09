// Copyright (c) 2021 Samsung Research America
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "path_utils/path_utils.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_controller/plugins/feasible_path_handler.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"

class BasicAPIRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  BasicAPIRPP()
  : nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController() {}

  double getSpeed() {return params_->max_linear_vel;}

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsgWrapper(
    const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return createCarrotMsg(carrot_pose);
  }

  void setVelocityScaledLookAhead() {params_->use_velocity_scaled_lookahead_dist = true;}
  void setCostRegulationScaling() {params_->use_cost_regulated_linear_velocity_scaling = true;}
  void resetVelocityRegulationScaling() {params_->use_regulated_linear_velocity_scaling = false;}

  double getLookAheadDistanceWrapper(const geometry_msgs::msg::Twist & twist)
  {
    return getLookAheadDistance(twist);
  }

  bool shouldRotateToPathWrapper(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
  {
    double x_vel_sign = 1.0;
    return shouldRotateToPath(carrot_pose, angle_to_path, x_vel_sign);
  }

  bool shouldRotateToGoalHeadingWrapper(const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return shouldRotateToGoalHeading(carrot_pose);
  }

  void rotateToHeadingWrapper(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
  {
    return rotateToHeading(linear_vel, angular_vel, angle_to_path, curr_speed);
  }

  void applyConstraintsWrapper(
    const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
    const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
  {
    return applyConstraints(
      curvature, curr_speed, pose_cost, path,
      linear_vel, sign);
  }
};

TEST(RegulatedPurePursuitTest, basicAPI)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");

  // instantiate
  auto ctrl = std::make_shared<BasicAPIRPP>();
  costmap->on_configure(rclcpp_lifecycle::State());
  ctrl->configure(node, name, tf, costmap);
  ctrl->activate();
  ctrl->deactivate();
  ctrl->cleanup();

  // set speed limit
  const double base_speed = ctrl->getSpeed();
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(0.51, false);
  EXPECT_EQ(ctrl->getSpeed(), 0.51);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(30, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed * 0.3);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
}

TEST(RegulatedPurePursuitTest, createCarrotMsg)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "Hi!";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 12.0;
  pose.pose.orientation.w = 0.5;

  auto rtn = ctrl->createCarrotMsgWrapper(pose);
  EXPECT_EQ(rtn->header.frame_id, std::string("Hi!"));
  EXPECT_EQ(rtn->point.x, 1.0);
  EXPECT_EQ(rtn->point.y, 12.0);
  EXPECT_EQ(rtn->point.z, 0.01);
}

TEST(RegulatedPurePursuitTest, lookaheadAPI)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<nav2::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::Twist twist;

  // test getLookAheadDistance
  double rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);  // default lookahead_dist

  // shouldn't be a function of speed
  twist.linear.x = 10.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);

  // now it should be a function of velocity, max out
  ctrl->setVelocityScaledLookAhead();
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.9);  // 10 speed maxes out at max_lookahead_dist

  // check normal range
  twist.linear.x = 0.35;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_NEAR(rtn, 0.525, 0.0001);  // 1.5 * 0.35

  // check minimum range
  twist.linear.x = 0.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.3);
}

TEST(RegulatedPurePursuitTest, rotateTests)
{
  // --------------------------
  // Non-Stateful Configuration
  // --------------------------
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<nav2::LifecycleNode>("testRPP");
  nav2::declare_parameter_if_not_declared(
    node, "PathFollower.stateful", rclcpp::ParameterValue(false));

  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  // shouldRotateToPath
  geometry_msgs::msg::PoseStamped carrot;
  double angle_to_path_rtn;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.25;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 1.0;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), true);

  // shouldRotateToGoalHeading
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.0;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.24;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.26;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), false);

  // rotateToHeading
  double lin_v = 10.0;
  double ang_v = 0.5;
  double angle_to_path = 0.4;
  geometry_msgs::msg::Twist curr_speed;
  curr_speed.angular.z = 1.75;

  // basic full speed at a speed
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(lin_v, 0.0);
  EXPECT_EQ(ang_v, 1.6);  // hit slow down limit

  // negative direction
  angle_to_path = -0.4;
  curr_speed.angular.z = -1.75;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(ang_v, -1.6);  // hit slow down limit

  // kinematic clamping, no speed, some speed accelerating, some speed decelerating
  angle_to_path = 0.4;
  curr_speed.angular.z = 0.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.16, 0.01);

  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 1.16, 0.01);

  angle_to_path = -0.4;
  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.84, 0.01);

  // -----------------------
  // Stateful Configuration
  // -----------------------
  node->set_parameter(
    rclcpp::Parameter("PathFollower.stateful", true));

  ctrl->configure(node, name, tf, costmap);

  // Start just outside tolerance
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.26;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), false);

  // Enter tolerance (should set internal flag)
  carrot.pose.position.y = 0.24;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);

  // Move outside tolerance again - still expect true (due to persistent state)
  carrot.pose.position.y = 0.26;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);
}

TEST(RegulatedPurePursuitTest, applyConstraints)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<nav2::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  constexpr double approach_velocity_scaling_dist = 0.6;
  nav2::declare_parameter_if_not_declared(
    node,
    name + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(approach_velocity_scaling_dist));

  ctrl->configure(node, name, tf, costmap);

  auto no_approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(approach_velocity_scaling_dist + 1.0)
  });

  double curvature = 0.5;
  geometry_msgs::msg::Twist curr_speed;
  double pose_cost = 0.0;
  double linear_vel = 0.0;
  double sign = 1.0;

  // test curvature regulation (default)
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, no_approach_path,
    linear_vel, sign);
  EXPECT_EQ(linear_vel, 0.25);  // min set speed

  linear_vel = 1.0;
  curvature = 0.7407;
  curr_speed.linear.x = 0.5;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, no_approach_path,
    linear_vel, sign);
  EXPECT_NEAR(linear_vel, 0.5, 0.01);  // lower by curvature

  linear_vel = 1.0;
  curvature = 1000.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, no_approach_path,
    linear_vel, sign);
  EXPECT_NEAR(linear_vel, 0.25, 0.01);  // min out by curvature

  // Approach velocity scaling on a path with no distance left
  auto approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(0.0)
  });

  linear_vel = 1.0;
  curvature = 0.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, approach_path,
    linear_vel, sign);
  EXPECT_NEAR(linear_vel, 0.05, 0.01);  // min out on min approach velocity

  // now try with cost regulation (turn off velocity and only cost)
  // ctrl->setCostRegulationScaling();
  // ctrl->resetVelocityRegulationScaling();
  // curvature = 0.0;

  // min changeable cost
  // pose_cost = 1;
  // linear_vel = 0.5;
  // curr_speed.linear.x = 0.5;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.498, 0.01);

  // max changing cost
  // pose_cost = 127;
  // curr_speed.linear.x = 0.255;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.255, 0.01);

  // over max cost thresh
  // pose_cost = 200;
  // curr_speed.linear.x = 0.25;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.25, 0.01);

  // test kinematic clamping
  // pose_cost = 200;
  // curr_speed.linear.x = 1.0;
  // ctrl->applyConstraintsWrapper(
  //   dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.5, 0.01);
}

TEST(RegulatedPurePursuitTest, testDynamicParameter)
{
  auto node = std::make_shared<nav2::LifecycleNode>("Smactest");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto ctrl =
    std::make_unique<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  ctrl->configure(node, "test", tf, costmap);
  ctrl->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.max_linear_vel", 1.0),
      rclcpp::Parameter("test.min_linear_vel", -1.0),
      rclcpp::Parameter("test.max_angular_vel", 2.0),
      rclcpp::Parameter("test.min_angular_vel", -2.0),
      rclcpp::Parameter("test.max_linear_accel", 2.0),
      rclcpp::Parameter("test.max_linear_decel", -2.0),
      rclcpp::Parameter("test.max_angular_accel", 3.0),
      rclcpp::Parameter("test.max_angular_decel", -3.0),
      rclcpp::Parameter("test.lookahead_dist", 7.0),
      rclcpp::Parameter("test.max_lookahead_dist", 7.0),
      rclcpp::Parameter("test.min_lookahead_dist", 6.0),
      rclcpp::Parameter("test.lookahead_time", 1.8),
      rclcpp::Parameter("test.rotate_to_heading_angular_vel", 18.0),
      rclcpp::Parameter("test.min_approach_linear_velocity", 1.0),
      rclcpp::Parameter("test.approach_velocity_scaling_dist", 0.8),
      rclcpp::Parameter("test.max_allowed_time_to_collision_up_to_carrot", 2.0),
      rclcpp::Parameter("test.min_distance_to_obstacle", 2.0),
      rclcpp::Parameter("test.cost_scaling_dist", 2.0),
      rclcpp::Parameter("test.cost_scaling_gain", 4.0),
      rclcpp::Parameter("test.regulated_linear_scaling_min_radius", 10.0),
      rclcpp::Parameter("test.rotate_to_heading_min_angle", 0.7),
      rclcpp::Parameter("test.regulated_linear_scaling_min_speed", 4.0),
      rclcpp::Parameter("test.use_velocity_scaled_lookahead_dist", false),
      rclcpp::Parameter("test.use_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.use_cost_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.inflation_cost_scaling_factor", 1.0),
      rclcpp::Parameter("test.allow_reversing", false),
      rclcpp::Parameter("test.use_rotate_to_heading", false),
      rclcpp::Parameter("test.stateful", false),
      rclcpp::Parameter("test.use_dynamic_window", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.max_linear_vel").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.min_linear_vel").as_double(), -1.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_vel").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.min_angular_vel").as_double(), -2.0);
  EXPECT_EQ(node->get_parameter("test.max_linear_accel").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.max_linear_decel").as_double(), -2.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_accel").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_decel").as_double(), -3.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.max_lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.min_lookahead_dist").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_time").as_double(), 1.8);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_angular_vel").as_double(), 18.0);
  EXPECT_EQ(node->get_parameter("test.min_approach_linear_velocity").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.approach_velocity_scaling_dist").as_double(), 0.8);
  EXPECT_EQ(
    node->get_parameter(
      "test.max_allowed_time_to_collision_up_to_carrot").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.min_distance_to_obstacle").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_dist").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_gain").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.regulated_linear_scaling_min_radius").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_min_angle").as_double(), 0.7);
  EXPECT_EQ(node->get_parameter("test.regulated_linear_scaling_min_speed").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.use_velocity_scaled_lookahead_dist").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_regulated_linear_velocity_scaling").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.inflation_cost_scaling_factor").as_double(), 1.0);
  EXPECT_EQ(
    node->get_parameter(
      "test.use_cost_regulated_linear_velocity_scaling").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.allow_reversing").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_rotate_to_heading").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.stateful").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_dynamic_window").as_bool(), true);

  // Should fail
  auto results2 = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.inflation_cost_scaling_factor", -1.0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results2);

  auto results3 = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.use_rotate_to_heading", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results3);

  auto results4 = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.allow_reversing", false),
      rclcpp::Parameter("test.use_rotate_to_heading", true),
      rclcpp::Parameter("test.allow_reversing", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results4);
}

TEST(RegulatedPurePursuitTest, computeVelocityByDWPP)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<nav2::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  // Enable DWPP so computeVelocityCommands executes the dynamic window branch.
  nav2::declare_parameter_if_not_declared(
    node, name + ".use_dynamic_window", rclcpp::ParameterValue(false));
  node->set_parameter(rclcpp::Parameter(name + ".use_dynamic_window", true));

  // Disable collision detection to simplify test.
  nav2::declare_parameter_if_not_declared(
    node, name + ".use_collision_detection", rclcpp::ParameterValue(true));
  node->set_parameter(rclcpp::Parameter(name + ".use_collision_detection", false));

  ctrl->configure(node, name, tf, costmap);
  ctrl->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap, tf);

  auto stamp = node->get_clock()->now();

  // Simple straight path ahead of the robot in map frame.
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = "map";
  start_pose.header.stamp = stamp;
  start_pose.pose.orientation.w = 1.0;
  auto plan = path_utils::generate_path(
    start_pose, 0.1,
    {std::make_unique<path_utils::Straight>(2.0)});
  ctrl->newPathReceived(plan);
  path_handler.setPlan(plan);

  // Provide transform into base frame so the controller can compute commands.
  geometry_msgs::msg::TransformStamped map_to_base;
  map_to_base.header.stamp = stamp;
  map_to_base.header.frame_id = "map";
  map_to_base.child_frame_id = "base_link";
  map_to_base.transform.rotation.w = 1.0;
  tf->setTransform(map_to_base, "dwpp-controller-test");

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist current_speed;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap);

  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(
    closest_point, pruned_plan_end);
  auto goal = path_handler.getTransformedGoal(robot_pose.header.stamp);
  auto cmd_vel = ctrl->computeVelocityCommands(
    robot_pose, current_speed, &checker, transformed_global_plan, goal);

  EXPECT_EQ(cmd_vel.twist.linear.x, 0.125);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.0);

  ctrl->deactivate();
  ctrl->cleanup();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
