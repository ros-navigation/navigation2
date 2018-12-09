/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Dave Hershberger
*********************************************************************/
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class FootprintTestNode : public nav2_costmap_2d::Costmap2DROS
{
public:
  FootprintTestNode(std::string name, tf2_ros::Buffer & buffer)
  : nav2_costmap_2d::Costmap2DROS(name, buffer)
  {}

  void testFootprint(double footprint_padding, std::string footprint)
  {
    footprint_padding_ = footprint_padding;
    if (footprint != "" && footprint != "[]") {
      std::vector<geometry_msgs::msg::Point> new_footprint;
      if (nav2_costmap_2d::makeFootprintFromString(footprint, new_footprint)) {
        nav2_costmap_2d::Costmap2DROS::setUnpaddedRobotFootprint(new_footprint);
      } else {
        RCLCPP_ERROR(get_logger(), "Invalid footprint string");
      }
    }
  }
  void testFootprint(double footprint_padding, double robot_radius)
  {
    footprint_padding_ = footprint_padding;
    nav2_costmap_2d::Costmap2DROS::setUnpaddedRobotFootprint(
      nav2_costmap_2d::makeFootprintFromRadius(robot_radius));
  }
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
  auto node = rclcpp::Node::make_shared("footprint_tests");

  tf_ = new tf2_ros::Buffer(node->get_clock());
  tfl_ = new tf2_ros::TransformListener(*tf_);

  // This empty transform is added to satisfy the constructor of
  // Costmap2DROS, which waits for the transform from map to base_link
  // to become available.
  geometry_msgs::msg::TransformStamped base_rel_map;
  base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
  base_rel_map.child_frame_id = "base_link";
  base_rel_map.header.frame_id = "map";
  base_rel_map.header.stamp = node->now();
  tf_->setTransform(base_rel_map, "footprint_tests");

  costmap_ = new FootprintTestNode("costmap_footprint_tests", *tf_);
  }

protected:
  FootprintTestNode * costmap_;
  tf2_ros::TransformListener * tfl_;
  tf2_ros::Buffer * tf_;
};

// Start with empty test before updating test footprints
TEST_F(TestNode, footprint_empty)
{
  //FootprintTestNode cm("costmap_footprint_empty", *tf_);
  std::vector<geometry_msgs::msg::Point> footprint = costmap_->getRobotFootprint();
  // With no specification of footprint or radius,
  // defaults to 0.1 meter radius plus 0.01 meter padding.
  EXPECT_EQ(16, footprint.size());

  EXPECT_NEAR(0.11f, footprint[0].x, 0.0001);
  EXPECT_NEAR(0.0f, footprint[0].y, 0.0001);
  EXPECT_EQ(0.0f, footprint[0].z);
}

TEST_F(TestNode, unpadded_footprint_from_string_param)
{
  costmap_->testFootprint(0.0, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = costmap_->getRobotFootprint();
  EXPECT_EQ(3, footprint.size());

  EXPECT_EQ(1.0f, footprint[0].x);
  EXPECT_EQ(1.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(-1.0f, footprint[1].x);
  EXPECT_EQ(1.0f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(-1.0f, footprint[2].x);
  EXPECT_EQ(-1.0f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

TEST_F(TestNode, padded_footprint_from_string_param)
{
  costmap_->testFootprint(0.5, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = costmap_->getRobotFootprint();
  EXPECT_EQ(3, footprint.size());

  EXPECT_EQ(1.5f, footprint[0].x);
  EXPECT_EQ(1.5f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(-1.5f, footprint[1].x);
  EXPECT_EQ(1.5f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(-1.5f, footprint[2].x);
  EXPECT_EQ(-1.5f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

TEST_F(TestNode, radius_param)
{
  costmap_->testFootprint(0, 10.0);
  std::vector<geometry_msgs::msg::Point> footprint = costmap_->getRobotFootprint();
  // Circular robot has 16-point footprint auto-generated.
  EXPECT_EQ(16, footprint.size());

  // Check the first point
  EXPECT_EQ(10.0f, footprint[0].x);
  EXPECT_EQ(0.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  // Check the 4th point, which should be 90 degrees around the circle from the first.
  EXPECT_NEAR(0.0f, footprint[4].x, 0.0001);
  EXPECT_NEAR(10.0f, footprint[4].y, 0.0001);
  EXPECT_EQ(0.0f, footprint[4].z);
}

TEST_F(TestNode, footprint_from_same_level_param)
{
  costmap_->testFootprint(0.0, "[[1, 2], [3, 4], [5, 6]]");
  std::vector<geometry_msgs::msg::Point> footprint = costmap_->getRobotFootprint();
  EXPECT_EQ(3, footprint.size());

  EXPECT_EQ(1.0f, footprint[0].x);
  EXPECT_EQ(2.0f, footprint[0].y);
  EXPECT_EQ(0.0f, footprint[0].z);

  EXPECT_EQ(3.0f, footprint[1].x);
  EXPECT_EQ(4.0f, footprint[1].y);
  EXPECT_EQ(0.0f, footprint[1].z);

  EXPECT_EQ(5.0f, footprint[2].x);
  EXPECT_EQ(6.0f, footprint[2].y);
  EXPECT_EQ(0.0f, footprint[2].z);
}

/* int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("footprint_tests");

  tf_ = new tf2_ros::Buffer(node->get_clock());
  tfl_ = new tf2_ros::TransformListener(*tf_);

  // This empty transform is added to satisfy the constructor of
  // Costmap2DROS, which waits for the transform from map to base_link
  // to become available.
  geometry_msgs::msg::TransformStamped base_rel_map;
  base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
  base_rel_map.child_frame_id = "base_link";
  base_rel_map.header.frame_id = "map";
  base_rel_map.header.stamp = node->now();
  tf_->setTransform(base_rel_map, "footprint_tests");

  costmap_ = new FootprintTestNode("costmap_footprint_tests", *tf_);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} */
