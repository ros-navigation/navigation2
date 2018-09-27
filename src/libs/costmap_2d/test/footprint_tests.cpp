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
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

using namespace costmap_2d;

tf2_ros::TransformListener* tfl_;
tf2_ros::Buffer* tf_;

TEST( Costmap2DROS, unpadded_footprint_from_string_param )
{
  Costmap2DROS cm( "unpadded/string", *tf_ );
  std::vector<geometry_msgs::Point> footprint = cm.getRobotFootprint();
  EXPECT_EQ( 3, footprint.size() );

  EXPECT_EQ( 1.0f, footprint[ 0 ].x );
  EXPECT_EQ( 1.0f, footprint[ 0 ].y );
  EXPECT_EQ( 0.0f, footprint[ 0 ].z );

  EXPECT_EQ( -1.0f, footprint[ 1 ].x );
  EXPECT_EQ( 1.0f, footprint[ 1 ].y );
  EXPECT_EQ( 0.0f, footprint[ 1 ].z );

  EXPECT_EQ( -1.0f, footprint[ 2 ].x );
  EXPECT_EQ( -1.0f, footprint[ 2 ].y );
  EXPECT_EQ( 0.0f, footprint[ 2 ].z );
}

TEST( Costmap2DROS, padded_footprint_from_string_param )
{
  Costmap2DROS cm( "padded/string", *tf_ );
  std::vector<geometry_msgs::Point> footprint = cm.getRobotFootprint();
  EXPECT_EQ( 3, footprint.size() );

  EXPECT_EQ( 1.5f, footprint[ 0 ].x );
  EXPECT_EQ( 1.5f, footprint[ 0 ].y );
  EXPECT_EQ( 0.0f, footprint[ 0 ].z );

  EXPECT_EQ( -1.5f, footprint[ 1 ].x );
  EXPECT_EQ( 1.5f, footprint[ 1 ].y );
  EXPECT_EQ( 0.0f, footprint[ 1 ].z );

  EXPECT_EQ( -1.5f, footprint[ 2 ].x );
  EXPECT_EQ( -1.5f, footprint[ 2 ].y );
  EXPECT_EQ( 0.0f, footprint[ 2 ].z );
}

TEST( Costmap2DROS, radius_param )
{
  Costmap2DROS cm( "radius/sub", *tf_ );
  std::vector<geometry_msgs::Point> footprint = cm.getRobotFootprint();
  // Circular robot has 16-point footprint auto-generated.
  EXPECT_EQ( 16, footprint.size() );

  // Check the first point
  EXPECT_EQ( 10.0f, footprint[ 0 ].x );
  EXPECT_EQ( 0.0f, footprint[ 0 ].y );
  EXPECT_EQ( 0.0f, footprint[ 0 ].z );

  // Check the 4th point, which should be 90 degrees around the circle from the first.
  EXPECT_NEAR( 0.0f, footprint[ 4 ].x, 0.0001 );
  EXPECT_NEAR( 10.0f, footprint[ 4 ].y, 0.0001 );
  EXPECT_EQ( 0.0f, footprint[ 4 ].z );
}

TEST( Costmap2DROS, footprint_from_xmlrpc_param )
{
  Costmap2DROS cm( "xmlrpc", *tf_ );
  std::vector<geometry_msgs::Point> footprint = cm.getRobotFootprint();
  EXPECT_EQ( 4, footprint.size() );

  EXPECT_EQ( 0.1f, footprint[ 0 ].x );
  EXPECT_EQ( 0.1f, footprint[ 0 ].y );
  EXPECT_EQ( 0.0f, footprint[ 0 ].z );

  EXPECT_EQ( -0.1f, footprint[ 1 ].x );
  EXPECT_EQ( 0.1f, footprint[ 1 ].y );
  EXPECT_EQ( 0.0f, footprint[ 1 ].z );

  EXPECT_EQ( -0.1f, footprint[ 2 ].x );
  EXPECT_EQ( -0.1f, footprint[ 2 ].y );
  EXPECT_EQ( 0.0f, footprint[ 2 ].z );

  EXPECT_EQ( 0.1f, footprint[ 3 ].x );
  EXPECT_EQ( -0.1f, footprint[ 3 ].y );
  EXPECT_EQ( 0.0f, footprint[ 3 ].z );
}

TEST( Costmap2DROS, footprint_from_same_level_param )
{
  Costmap2DROS cm( "same_level", *tf_ );
  std::vector<geometry_msgs::Point> footprint = cm.getRobotFootprint();
  EXPECT_EQ( 3, footprint.size() );

  EXPECT_EQ( 1.0f, footprint[ 0 ].x );
  EXPECT_EQ( 2.0f, footprint[ 0 ].y );
  EXPECT_EQ( 0.0f, footprint[ 0 ].z );

  EXPECT_EQ( 3.0f, footprint[ 1 ].x );
  EXPECT_EQ( 4.0f, footprint[ 1 ].y );
  EXPECT_EQ( 0.0f, footprint[ 1 ].z );

  EXPECT_EQ( 5.0f, footprint[ 2 ].x );
  EXPECT_EQ( 6.0f, footprint[ 2 ].y );
  EXPECT_EQ( 0.0f, footprint[ 2 ].z );
}

TEST( Costmap2DROS, footprint_from_xmlrpc_param_failure )
{
  ASSERT_ANY_THROW( Costmap2DROS cm( "xmlrpc_fail", *tf_ ));
}

TEST( Costmap2DROS, footprint_empty )
{
  Costmap2DROS cm( "empty", *tf_ );
  std::vector<geometry_msgs::Point> footprint = cm.getRobotFootprint();
  // With no specification of footprint or radius, defaults to 0.46 meter radius plus 0.01 meter padding.
  EXPECT_EQ( 16, footprint.size() );

  EXPECT_NEAR( 0.47f, footprint[ 0 ].x, 0.0001 );
  EXPECT_NEAR( 0.0f, footprint[ 0 ].y, 0.0001 );
  EXPECT_EQ( 0.0f, footprint[ 0 ].z );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "footprint_tests_node");

  tf_ = new tf2_ros::Buffer( ros::Duration( 10 ));
  tfl_ = new tf2_ros::TransformListener(*tf_);

  // This empty transform is added to satisfy the constructor of
  // Costmap2DROS, which waits for the transform from map to base_link
  // to become available.
  geometry_msgs::TransformStamped base_rel_map;
  base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
  base_rel_map.child_frame_id = "base_link";
  base_rel_map.header.frame_id = "map";
  base_rel_map.header.stamp = ros::Time::now();
  tf_->setTransform( base_rel_map, "footprint_tests" );

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
