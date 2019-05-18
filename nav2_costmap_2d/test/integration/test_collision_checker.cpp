// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;

class TestCollisionChecker : public nav2_costmap_2d::CollisionChecker
{
public:
  TestCollisionChecker(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub,
    tf2_ros::Buffer & tf_buffer)
  : CollisionChecker(node, costmap_sub, footprint_sub, tf_buffer),
    node_(node),
    layers_("frame", false, false),
    costmap_received_(false),
    tf_(tf_buffer)
  {
    node_->get_parameter_or<std::string>("global_frame", global_frame_, std::string("map"));

    base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
    base_rel_map.child_frame_id = "base_link";
    base_rel_map.header.frame_id = "map";
    base_rel_map.header.stamp = node_->now();
    tf_.setTransform(base_rel_map, "collision_checker_test");

    // Add Static Layer
    nav2_costmap_2d::StaticLayer * slayer = new nav2_costmap_2d::StaticLayer();
    layers_.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(slayer));
    slayer->initialize(&layers_, "static", &tf_, node_);

    // Add Inflation Layer
    nav2_costmap_2d::InflationLayer * ilayer = new nav2_costmap_2d::InflationLayer();
    ilayer->initialize(&layers_, "inflation", &tf_, node_);
    std::shared_ptr<nav2_costmap_2d::Layer> ipointer(ilayer);
    layers_.addPlugin(ipointer);

    footprint_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "footprint", rmw_qos_profile_default);

    costmap_pub_ = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(node_,
        layers_.getCostmap(), global_frame_, "costmap", true);

    publish();
  }

  ~TestCollisionChecker() {}

  bool testPose(double x, double y, double theta)
  {
    geometry_msgs::msg::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    setPose(x, y, theta);
    costmap_received_ = false;

    while (!costmap_received_) {
      rclcpp::spin_some(node_);
    }

    return isCollisionFree(pose);
  }

  void setFootprint(double footprint_padding, double robot_radius)
  {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    new_footprint = nav2_costmap_2d::makeFootprintFromRadius(robot_radius);
    nav2_costmap_2d::padFootprint(new_footprint, footprint_padding);
    footprint_ = new_footprint;
    layers_.setFootprint(footprint_);
  }

protected:
  void setPose(double x, double y, double theta)
  {
    x_ = x;
    y_ = y;
    yaw_ = theta;

    geometry_msgs::msg::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    pose.orientation = tf2::toMsg(q);

    tf2::Transform transform;
    tf2::fromMsg(pose, transform);
    base_rel_map.transform = tf2::toMsg(transform);
    base_rel_map.header.stamp = node_->now();
    tf_.setTransform(base_rel_map, "collision_checker_test");
  }

  void publish()
  {
    auto timer_callback = [this]() -> void
      {
        try {
          costmap_sub_->getCostmap();
          costmap_received_ = true;
        } catch (const std::runtime_error & e) {
          costmap_received_ = false;
        }
        publishFootprint();
        publishCostmap();
      };

    timer_ = node_->create_wall_timer(0.1s, timer_callback);
  }

  void publishFootprint()
  {
    geometry_msgs::msg::PolygonStamped oriented_footprint;
    oriented_footprint.header.frame_id = global_frame_;
    oriented_footprint.header.stamp = node_->now();
    nav2_costmap_2d::transformFootprint(x_, y_, yaw_, footprint_, oriented_footprint);
    footprint_pub_->publish(oriented_footprint);
  }

  void publishCostmap()
  {
    layers_.updateMap(x_, y_, yaw_);
    costmap_pub_->publishCostmap();
  }

  double x_, y_, yaw_;
  rclcpp::Node::SharedPtr node_;
  nav2_costmap_2d::LayeredCostmap layers_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::string global_frame_;
  bool costmap_received_;
  geometry_msgs::msg::TransformStamped base_rel_map;
  tf2_ros::Buffer & tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {

    node_ = rclcpp::Node::make_shared(
      "test_collision_checker", nav2_util::get_node_options_default());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string costmap_topic = "costmap_raw";
    std::string footprint_topic = "footprint";
    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node_, costmap_topic);
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(node_, footprint_topic);

    collision_checker_ = std::make_unique<TestCollisionChecker>(node_, costmap_sub_, footprint_sub_,
        *tf_buffer_);
  }

  ~TestNode() {}

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::unique_ptr<TestCollisionChecker> collision_checker_;
};

TEST_F(TestNode, uknownSpace)
{
  collision_checker_->setFootprint(0, 1);

  // Completely off map
  ASSERT_EQ(collision_checker_->testPose(5, 13, 0), false);

  // Partially off map
  ASSERT_EQ(collision_checker_->testPose(5, 9.5, 0), false);

  // In unknown region inside map
  ASSERT_EQ(collision_checker_->testPose(2, 4, 0), false);

}

TEST_F(TestNode, FreeSpace)
{
  collision_checker_->setFootprint(0, 1);

  // In complete free space
  ASSERT_EQ(collision_checker_->testPose(2, 8.5, 0), true);

  // Partially in inscribed space
  ASSERT_EQ(collision_checker_->testPose(2.5, 7, 0), true);
}

TEST_F(TestNode, CollisionSpace)
{
  collision_checker_->setFootprint(0, 1);

  // Completely in obstacle
  ASSERT_EQ(collision_checker_->testPose(8.5, 6.5, 0), false);

  // Partially in obstacle
  ASSERT_EQ(collision_checker_->testPose(4.5, 4.5, 0), false);

}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(0, nullptr);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
