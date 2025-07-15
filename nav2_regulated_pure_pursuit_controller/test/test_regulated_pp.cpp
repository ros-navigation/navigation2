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
#include "nav2_util/lifecycle_node.hpp"
#include "path_utils/path_utils.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/controller_exceptions.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class BasicAPIRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  BasicAPIRPP()
  : nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController() {}

  nav_msgs::msg::Path getPlan() {return path_handler_->getPlan();}

  double getSpeed() {return params_->desired_linear_vel;}

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

  static geometry_msgs::msg::Point circleSegmentIntersectionWrapper(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r)
  {
    return circleSegmentIntersection(p1, p2, r);
  }

  geometry_msgs::msg::PoseStamped
  projectCarrotPastGoalWrapper(
    const double & dist,
    const nav_msgs::msg::Path & path)
  {
    return getLookAheadPoint(dist, path, true);
  }

  geometry_msgs::msg::PoseStamped getLookAheadPointWrapper(
    const double & dist, const nav_msgs::msg::Path & path)
  {
    return getLookAheadPoint(dist, path);
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

  double findVelocitySignChangeWrapper(
    const nav_msgs::msg::Path & transformed_plan)
  {
    return findVelocitySignChange(transformed_plan);
  }

  nav_msgs::msg::Path transformGlobalPlanWrapper(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist);
  }
};

TEST(RegulatedPurePursuitTest, basicAPI)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
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

  // setPlan and get plan
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].header.frame_id = "fake_frame";
  ctrl->setPlan(path);
  EXPECT_EQ(ctrl->getPlan().poses.size(), 2ul);
  EXPECT_EQ(ctrl->getPlan().poses[0].header.frame_id, std::string("fake_frame"));

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

TEST(RegulatedPurePursuitTest, findVelocitySignChange)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPPfindVelocitySignChange");
  auto ctrl = std::make_shared<BasicAPIRPP>();

  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "smb";
  auto time = node->get_clock()->now();
  pose.header.stamp = time;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  path.poses.resize(3);
  path.header.frame_id = "smb";
  path.header.stamp = pose.header.stamp;
  path.poses[0].pose.position.x = 1.0;
  path.poses[0].pose.position.y = 1.0;
  path.poses[1].pose.position.x = 2.0;
  path.poses[1].pose.position.y = 2.0;
  path.poses[2].pose.position.x = -1.0;
  path.poses[2].pose.position.y = -1.0;
  ctrl->setPlan(path);
  auto rtn = ctrl->findVelocitySignChangeWrapper(path);
  EXPECT_EQ(rtn, sqrt(8.0));

  path.poses[2].pose.position.x = 3.0;
  path.poses[2].pose.position.y = 3.0;
  ctrl->setPlan(path);
  rtn = ctrl->findVelocitySignChangeWrapper(path);
  EXPECT_EQ(rtn, std::numeric_limits<double>::max());
}

using CircleSegmentIntersectionParam = std::tuple<
  std::pair<double, double>,
  std::pair<double, double>,
  double,
  std::pair<double, double>
>;

class CircleSegmentIntersectionTest
  : public ::testing::TestWithParam<CircleSegmentIntersectionParam>
{};

TEST_P(CircleSegmentIntersectionTest, circleSegmentIntersection)
{
  auto pair1 = std::get<0>(GetParam());
  auto pair2 = std::get<1>(GetParam());
  auto r = std::get<2>(GetParam());
  auto expected_pair = std::get<3>(GetParam());
  auto pair_to_point = [](std::pair<double, double> p) -> geometry_msgs::msg::Point {
      geometry_msgs::msg::Point point;
      point.x = p.first;
      point.y = p.second;
      point.z = 0.0;
      return point;
    };
  auto p1 = pair_to_point(pair1);
  auto p2 = pair_to_point(pair2);
  auto actual = BasicAPIRPP::circleSegmentIntersectionWrapper(p1, p2, r);
  auto expected_point = pair_to_point(expected_pair);
  EXPECT_DOUBLE_EQ(actual.x, expected_point.x);
  EXPECT_DOUBLE_EQ(actual.y, expected_point.y);
  // Expect that the intersection point is actually r away from the origin
  EXPECT_DOUBLE_EQ(r, std::hypot(actual.x, actual.y));
}

INSTANTIATE_TEST_SUITE_P(
  InterpolationTest,
  CircleSegmentIntersectionTest,
  testing::Values(
    // Origin to the positive X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {2.0, 0.0},
  1.0,
  {1.0, 0.0}
},
    // Origin to hte negative X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-2.0, 0.0},
  1.0,
  {-1.0, 0.0}
},
    // Origin to the positive Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  1.0,
  {0.0, 1.0}
},
    // Origin to the negative Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, -2.0},
  1.0,
  {0.0, -1.0}
},
    // non-origin to the X axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {4.0, 0.0},
  {-1.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // non-origin to the Y axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {0.0, 4.0},
  {0.0, -0.5},
  2.0,
  {0.0, 2.0}
},
    // origin to the positive X axis, on the circle
    CircleSegmentIntersectionParam{
  {2.0, 0.0},
  {0.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // origin to the positive Y axis, on the circle
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  2.0,
  {0.0, 2.0}
},
    // origin to the upper-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, 8.0},
  5.0,
  {3.0, 4.0}
},
    // origin to the lower-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, -8.0},
  5.0,
  {-3.0, -4.0}
},
    // origin to the upper-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, 8.0},
  5.0,
  {-3.0, 4.0}
},
    // origin to the lower-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, -8.0},
  5.0,
  {3.0, -4.0}
}
));

TEST(RegulatedPurePursuitTest, projectCarrotPastGoal) {
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  double EPSILON = std::numeric_limits<float>::epsilon();

  nav_msgs::msg::Path path;
  // More than 2 poses
  path.poses.resize(4);
  path.poses[0].pose.position.x = 0.0;
  path.poses[1].pose.position.x = 1.0;
  path.poses[2].pose.position.x = 2.0;
  path.poses[3].pose.position.x = 3.0;
  auto pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, 0.0, EPSILON);

  // 2 poses fwd
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 2.0;
  path.poses[1].pose.position.x = 3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, 0.0, EPSILON);

  // 2 poses at 45°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 2.0;
  path.poses[0].pose.position.y = 2.0;
  path.poses[1].pose.position.x = 3.0;
  path.poses[1].pose.position.y = 3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, cos(45.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(45.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at 90°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 2.0;
  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = 3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, cos(90.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(90.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at 135°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = -2.0;
  path.poses[0].pose.position.y = 2.0;
  path.poses[1].pose.position.x = -3.0;
  path.poses[1].pose.position.y = 3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, cos(135.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(135.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses bck
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = -2.0;
  path.poses[1].pose.position.x = -3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, -10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, 0.0, EPSILON);

  // 2 poses at -135°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = -2.0;
  path.poses[0].pose.position.y = -2.0;
  path.poses[1].pose.position.x = -3.0;
  path.poses[1].pose.position.y = -3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, cos(-135.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(-135.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at -90°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = -2.0;
  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = -3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, cos(-90.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(-90.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at -45°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 2.0;
  path.poses[0].pose.position.y = -2.0;
  path.poses[1].pose.position.x = 3.0;
  path.poses[1].pose.position.y = -3.0;
  pt = ctrl->projectCarrotPastGoalWrapper(10.0, path);
  EXPECT_NEAR(pt.pose.position.x, cos(-45.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(-45.0 * M_PI / 180) * 10.0, EPSILON);
}

TEST(RegulatedPurePursuitTest, lookaheadAPI)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
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

  // test getLookAheadPoint
  double dist = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i);
  }

  // test exact hits
  auto pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.0);

  // test interpolation
  ctrl->configure(node, name, tf, costmap);
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 3.8);
}

TEST(RegulatedPurePursuitTest, rotateTests)
{
  // --------------------------
  // Non-Stateful Configuration
  // --------------------------
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  nav2_util::declare_parameter_if_not_declared(
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  constexpr double approach_velocity_scaling_dist = 0.6;
  nav2_util::declare_parameter_if_not_declared(
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

  // min changable cost
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("Smactest");
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
    {rclcpp::Parameter("test.desired_linear_vel", 1.0),
      rclcpp::Parameter("test.lookahead_dist", 7.0),
      rclcpp::Parameter("test.max_lookahead_dist", 7.0),
      rclcpp::Parameter("test.min_lookahead_dist", 6.0),
      rclcpp::Parameter("test.lookahead_time", 1.8),
      rclcpp::Parameter("test.rotate_to_heading_angular_vel", 18.0),
      rclcpp::Parameter("test.min_approach_linear_velocity", 1.0),
      rclcpp::Parameter("test.max_allowed_time_to_collision_up_to_carrot", 2.0),
      rclcpp::Parameter("test.cost_scaling_dist", 2.0),
      rclcpp::Parameter("test.cost_scaling_gain", 4.0),
      rclcpp::Parameter("test.regulated_linear_scaling_min_radius", 10.0),
      rclcpp::Parameter("test.transform_tolerance", 30.0),
      rclcpp::Parameter("test.max_angular_accel", 3.0),
      rclcpp::Parameter("test.rotate_to_heading_min_angle", 0.7),
      rclcpp::Parameter("test.regulated_linear_scaling_min_speed", 4.0),
      rclcpp::Parameter("test.use_velocity_scaled_lookahead_dist", false),
      rclcpp::Parameter("test.use_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.use_cost_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.inflation_cost_scaling_factor", 1.0),
      rclcpp::Parameter("test.allow_reversing", false),
      rclcpp::Parameter("test.use_rotate_to_heading", false),
      rclcpp::Parameter("test.stateful", false)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.desired_linear_vel").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.max_lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.min_lookahead_dist").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_time").as_double(), 1.8);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_angular_vel").as_double(), 18.0);
  EXPECT_EQ(node->get_parameter("test.min_approach_linear_velocity").as_double(), 1.0);
  EXPECT_EQ(
    node->get_parameter(
      "test.max_allowed_time_to_collision_up_to_carrot").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_dist").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_gain").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.regulated_linear_scaling_min_radius").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.transform_tolerance").as_double(), 30.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_accel").as_double(), 3.0);
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

class TransformGlobalPlanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctrl_ = std::make_shared<BasicAPIRPP>();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  void configure_costmap(uint16_t width, double resolution)
  {
    constexpr char costmap_frame[] = "test_costmap_frame";
    constexpr char robot_frame[] = "test_robot_frame";

    auto results = costmap_->set_parameters(
    {
      rclcpp::Parameter("global_frame", costmap_frame),
      rclcpp::Parameter("robot_base_frame", robot_frame),
      rclcpp::Parameter("width", width),
      rclcpp::Parameter("height", width),
      rclcpp::Parameter("resolution", resolution)
    });
    for (const auto & result : results) {
      EXPECT_TRUE(result.successful) << result.reason;
    }

    rclcpp_lifecycle::State state;
    costmap_->on_configure(state);
  }

  void configure_controller(double max_robot_pose_search_dist)
  {
    std::string plugin_name = "test_rpp";
    nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name + ".max_robot_pose_search_dist",
      rclcpp::ParameterValue(max_robot_pose_search_dist));
    ctrl_->configure(node_, plugin_name, tf_buffer_, costmap_);
  }

  void setup_transforms(geometry_msgs::msg::Point & robot_position)
  {
    transform_time_ = node_->get_clock()->now();
    // Note: transforms go parent to child

    // We will have a separate path and costmap frame for completeness,
    // but we will leave them cooincident for convenience.
    geometry_msgs::msg::TransformStamped path_to_costmap;
    path_to_costmap.header.frame_id = PATH_FRAME;
    path_to_costmap.header.stamp = transform_time_;
    path_to_costmap.child_frame_id = COSTMAP_FRAME;
    path_to_costmap.transform.translation.x = 0.0;
    path_to_costmap.transform.translation.y = 0.0;
    path_to_costmap.transform.translation.z = 0.0;

    geometry_msgs::msg::TransformStamped costmap_to_robot;
    costmap_to_robot.header.frame_id = COSTMAP_FRAME;
    costmap_to_robot.header.stamp = transform_time_;
    costmap_to_robot.child_frame_id = ROBOT_FRAME;
    costmap_to_robot.transform.translation.x = robot_position.x;
    costmap_to_robot.transform.translation.y = robot_position.y;
    costmap_to_robot.transform.translation.z = robot_position.z;

    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms = {
      path_to_costmap,
      costmap_to_robot
    };
    for (const auto & transform : tf_message.transforms) {
      tf_buffer_->setTransform(transform, "test", false);
    }
    tf_buffer_->setUsingDedicatedThread(true);  // lying to let it do transforms
  }

  static constexpr char PATH_FRAME[] = "test_path_frame";
  static constexpr char COSTMAP_FRAME[] = "test_costmap_frame";
  static constexpr char ROBOT_FRAME[] = "test_robot_frame";

  std::shared_ptr<BasicAPIRPP> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Time transform_time_;
};

// This tests that not only should nothing get pruned on a costmap
// that contains the entire global_plan, and also that it doesn't skip to the end of the path
// which is closer to the robot pose than the start.
TEST_F(TransformGlobalPlanTest, no_pruning_on_large_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // A really big costmap
  // the max_costmap_extent should be 50m

  configure_costmap(100u, 0.1);
  configure_controller(5.0);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double spacing = 0.1;
  constexpr double circle_radius = 1.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan

  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_EQ(transformed_plan.poses.size(), global_plan.poses.size());
}

// This plan shouldn't get pruned because of the costmap,
// but should be half pruned because it is halfway around the circle
TEST_F(TransformGlobalPlanTest, transform_start_selection)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 4.0;  // on the other side of the circle
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A really big costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  // This should just be at least half the circumference: pi*r ~= 6
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), global_plan.poses.size() / 2, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// This should throw an exception when all poses are outside of the costmap
TEST_F(TransformGlobalPlanTest, all_poses_outside_of_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = 1000.0;
  robot_pose.pose.position.y = 1000.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(10u, 0.1);
  // This should just be at least half the circumference: pi*r ~= 6
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  EXPECT_THROW(ctrl_->transformGlobalPlanWrapper(robot_pose), nav2_core::ControllerException);
}

// Should shortcut the circle if the circle is shorter than max_robot_pose_search_dist
TEST_F(TransformGlobalPlanTest, good_circle_shortcut)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 0.1;
  constexpr double circle_radius = 2.0;  // diameter 4

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(100u, 0.1);
  // This should just be at least the circumference: 2*pi*r ~= 12
  constexpr double max_robot_pose_search_dist = 15.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::LeftCircle>(circle_radius)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), 1, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// Simple costmap pruning on a straight line
TEST_F(TransformGlobalPlanTest, costmap_pruning)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 1.0;

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(20u, 0.5);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double path_length = 100.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::Straight>(path_length)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  EXPECT_NEAR(transformed_plan.poses.size(), 10u, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}

// Should prune out later portions of the path that come back into the costmap
TEST_F(TransformGlobalPlanTest, prune_after_leaving_costmap)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = COSTMAP_FRAME;
  robot_pose.header.stamp = transform_time_;
  // far away from the path
  robot_pose.pose.position.x = -0.1;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  // Could set orientation going the other way, but RPP doesn't care
  constexpr double spacing = 1.0;

  // A "normal" costmap
  // the max_costmap_extent should be 50m
  configure_costmap(20u, 0.5);
  constexpr double max_robot_pose_search_dist = 10.0;
  configure_controller(max_robot_pose_search_dist);
  setup_transforms(robot_pose.pose.position);

  // Set up test path;

  geometry_msgs::msg::PoseStamped start_of_path;
  start_of_path.header.frame_id = PATH_FRAME;
  start_of_path.header.stamp = transform_time_;
  start_of_path.pose.position.x = 0.0;
  start_of_path.pose.position.y = 0.0;
  start_of_path.pose.position.z = 0.0;

  constexpr double path_length = 100.0;

  auto global_plan = path_utils::generate_path(
    start_of_path, spacing, {
    std::make_unique<path_utils::Straight>(path_length),
    std::make_unique<path_utils::LeftTurnAround>(1.0),
    std::make_unique<path_utils::Straight>(path_length)
  });

  ctrl_->setPlan(global_plan);

  // Transform the plan
  auto transformed_plan = ctrl_->transformGlobalPlanWrapper(robot_pose);
  // This should be essentially the same as the regular straight path
  EXPECT_NEAR(transformed_plan.poses.size(), 10u, 1);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.x, 0.0, 0.5);
  EXPECT_NEAR(transformed_plan.poses[0].pose.position.y, 0.0, 0.5);
}
