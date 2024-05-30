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
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/footprint.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class FootprintTestNode
{
public:
  FootprintTestNode()
  {
    // Default footprint padding and footprint radius from Costmap2DROS
    testFootprint(0.01f, 0.1);
  }

  ~FootprintTestNode() {}

  void testFootprint(double footprint_padding, std::string footprint)
  {
    footprint_padding_ = footprint_padding;
    if (footprint != "" && footprint != "[]") {
      std::vector<geometry_msgs::msg::Point> new_footprint;
      if (nav2_costmap_2d::makeFootprintFromString(footprint, new_footprint)) {
        setRobotFootprint(new_footprint);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("footprint_tester"), "Invalid footprint string");
      }
    }
  }

  void testFootprint(double footprint_padding, double robot_radius)
  {
    footprint_padding_ = footprint_padding;
    setRobotFootprint(nav2_costmap_2d::makeFootprintFromRadius(robot_radius));
  }

  std::vector<geometry_msgs::msg::Point> getRobotFootprint()
  {
    return footprint_;
  }

protected:
  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
  {
    footprint_ = points;
    nav2_costmap_2d::padFootprint(footprint_, footprint_padding_);
  }

  double footprint_padding_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    footprint_tester_ = std::make_shared<FootprintTestNode>();
  }

  ~TestNode() {}

protected:
  std::shared_ptr<FootprintTestNode> footprint_tester_;
};

// Start with empty test before updating test footprints
TEST_F(TestNode, footprint_empty)
{
  // FootprintTestNode cm("costmap_footprint_empty", *tf_);
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  // With no specification of footprint or radius,
  // defaults to 0.1 meter radius plus 0.01 meter padding.
  EXPECT_EQ(16u, footprint.size());

  EXPECT_NEAR(0.11f, footprint[0].x, 0.0001);
  EXPECT_NEAR(0.0f, footprint[0].y, 0.0001);
  EXPECT_EQ(0.0f, footprint[0].z);
}

TEST_F(TestNode, unpadded_footprint_from_string_param)
{
  footprint_tester_->testFootprint(0.0, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

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
  footprint_tester_->testFootprint(0.5, "[[1, 1], [-1, 1], [-1, -1]]");

  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

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
  footprint_tester_->testFootprint(0, 10.0);
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  // Circular robot has 16-point footprint auto-generated.
  EXPECT_EQ(16u, footprint.size());

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
  footprint_tester_->testFootprint(0.0, "[[1, 2], [3, 4], [5, 6]]");
  std::vector<geometry_msgs::msg::Point> footprint = footprint_tester_->getRobotFootprint();
  EXPECT_EQ(3u, footprint.size());

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
