/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Wilco Bonestroo
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_core/exceptions.hpp"

geometry_msgs::msg::Point rotate_origin(geometry_msgs::msg::Point p, double angle)
{
  double s = sin(angle);
  double c = cos(angle);

  // rotate point
  double xnew = p.x * c - p.y * s;
  double ynew = p.x * s + p.y * c;

  p.x = xnew;
  p.y = ynew;

  return p;
}

geometry_msgs::msg::Point p(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

double footprint_size_x_half = 1.8;
double footprint_size_y_half = 1.6;

std::vector<geometry_msgs::msg::Point> getFootprint()
{
  std::vector<geometry_msgs::msg::Point> footprint;
  footprint.push_back(p(footprint_size_x_half, footprint_size_y_half));
  footprint.push_back(p(footprint_size_x_half, -footprint_size_y_half));
  footprint.push_back(p(-footprint_size_x_half, -footprint_size_y_half));
  footprint.push_back(p(-footprint_size_x_half, footprint_size_y_half));
  return footprint;
}

TEST(ObstacleFootprint, GetOrientedFootprint)
{
  double theta = 0.1234;

  std::vector<geometry_msgs::msg::Point> footprint_before = getFootprint();

  std::vector<geometry_msgs::msg::Point> footprint_after;
  geometry_msgs::msg::Pose2D pose;
  pose.theta = theta;
  footprint_after = dwb_critics::getOrientedFootprint(pose, footprint_before);

  uint i;
  for (i = 0; i < footprint_before.size(); i++) {
    ASSERT_EQ(rotate_origin(footprint_before[i], theta), footprint_after[i]);
  }
}

TEST(ObstacleFootprint, Prepare)
{
  std::shared_ptr<dwb_critics::ObstacleFootprintCritic> critic =
    std::make_shared<dwb_critics::ObstacleFootprintCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  // costmap_ros->activate();
  costmap_ros->configure();
  std::string name = "test";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);

  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D vel;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D global_plan;

  // no footprint set in the costmap. Prepare should return false;
  std::vector<geometry_msgs::msg::Point> footprint;
  costmap_ros->setRobotFootprint(footprint);
  ASSERT_FALSE(critic->prepare(pose, vel, goal, global_plan));

  costmap_ros->setRobotFootprint(getFootprint());
  ASSERT_TRUE(critic->prepare(pose, vel, goal, global_plan));

  double epsilon = 0.01;
  // If the robot footprint goes of the map, it should trow an exception
  pose.x = footprint_size_x_half;  // This gives an error
  pose.y = footprint_size_y_half + epsilon;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = footprint_size_x_half + epsilon;
  pose.y = footprint_size_y_half;  // error
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() - footprint_size_x_half;  // error
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + footprint_size_y_half - epsilon;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() - footprint_size_x_half - epsilon;
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + footprint_size_y_half;  // error
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = footprint_size_x_half + epsilon;
  pose.y = footprint_size_y_half + epsilon;
  ASSERT_EQ(critic->scorePose(pose), 0.0);

  std::cout << "Origin x " << costmap_ros->getCostmap()->getOriginX() << std::endl;
  std::cout << "Origin y " << costmap_ros->getCostmap()->getOriginY() << std::endl;
  std::cout << "Resolution " << costmap_ros->getCostmap()->getResolution() << std::endl;
  std::cout << "Size in meters x " << costmap_ros->getCostmap()->getSizeInMetersX() << std::endl;
  std::cout << "Size in meters y " << costmap_ros->getCostmap()->getSizeInMetersY() << std::endl;
  std::cout << "Size in cells x " << costmap_ros->getCostmap()->getSizeInCellsX() << std::endl;
  std::cout << "Size in cells y " << costmap_ros->getCostmap()->getSizeInCellsY() << std::endl;

  for (unsigned int i = 1; i < costmap_ros->getCostmap()->getSizeInCellsX(); i++) {
    costmap_ros->getCostmap()->setCost(i, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  // It should now hit an obstacle (throw an expection)
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  /*

  // costmap_ros->shutdown();
  // costmap_ros->cleanup();
  node->shutdown();
  node->cleanup();
  */
  // costmap_ros->shutdown();
}

TEST(ObstacleFootprint, Prepare2)
{
  std::shared_ptr<dwb_critics::ObstacleFootprintCritic> critic =
    std::make_shared<dwb_critics::ObstacleFootprintCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  // costmap_ros->activate();
  costmap_ros->configure();
  std::string name = "test";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);

  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D vel;
  geometry_msgs::msg::Pose2D goal;
  nav_2d_msgs::msg::Path2D global_plan;

  // no footprint set in the costmap. Prepare should return false;
  std::vector<geometry_msgs::msg::Point> footprint;
  costmap_ros->setRobotFootprint(footprint);
  ASSERT_FALSE(critic->prepare(pose, vel, goal, global_plan));

  costmap_ros->setRobotFootprint(getFootprint());
  ASSERT_TRUE(critic->prepare(pose, vel, goal, global_plan));

  double epsilon = 0.01;
  // If the robot footprint goes of the map, it should trow an exception
  pose.x = footprint_size_x_half;  // This gives an error
  pose.y = footprint_size_y_half + epsilon;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = footprint_size_x_half + epsilon;
  pose.y = footprint_size_y_half;  // error
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() - footprint_size_x_half;  // error
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + footprint_size_y_half - epsilon;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() - footprint_size_x_half - epsilon;
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + footprint_size_y_half;  // error
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = footprint_size_x_half + epsilon;
  pose.y = footprint_size_y_half + epsilon;
  ASSERT_EQ(critic->scorePose(pose), 0.0);
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
