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
#include "nav2_costmap_2d/testing_helper.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  DummyCostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    std::string & topic_name)
  : CostmapSubscriber(node, topic_name)
  {}

  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    costmap_msg_ = msg;
    costmap_received_ = true;
  }
};

class DummyFootprintSubscriber : public nav2_costmap_2d::FootprintSubscriber
{
public:
  DummyFootprintSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    std::string & topic_name)
  : FootprintSubscriber(node, topic_name)
  {}

  void setFootprint(geometry_msgs::msg::PolygonStamped::SharedPtr msg)
  {
    footprint_ = msg;
    footprint_received_ = true;
  }
};

class TestCollisionChecker : public nav2_util::LifecycleNode
{
public:
  TestCollisionChecker(std::string name)
  : LifecycleNode(name, "", false),
    global_frame_("map")
  {
    // Declare non-plugin specific costmap parameters
    declare_parameter("map_topic", rclcpp::ParameterValue(std::string("/map")));
    declare_parameter("track_unknown_space", rclcpp::ParameterValue(true));
    declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    declare_parameter("unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
  }

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string costmap_topic = "costmap_raw";
    std::string footprint_topic = "published_footprint";

    costmap_sub_ = std::make_shared<DummyCostmapSubscriber>(
      shared_from_this(),
      costmap_topic);

    footprint_sub_ = std::make_shared<DummyFootprintSubscriber>(
      shared_from_this(),
      footprint_topic);

    collision_checker_ = std::make_unique<nav2_costmap_2d::CollisionChecker>(
      costmap_sub_, footprint_sub_, *tf_buffer_);

    base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
    base_rel_map.child_frame_id = "base_link";
    base_rel_map.header.frame_id = "map";
    base_rel_map.header.stamp = now();
    tf_buffer_->setTransform(base_rel_map, "collision_checker_test");

    layers_ = new nav2_costmap_2d::LayeredCostmap("frame", false, false);
    // Add Static Layer
    addStaticLayer(*layers_, *tf_buffer_, shared_from_this());
    // Add Inflation Layer
    addInflationLayer(*layers_, *tf_buffer_, shared_from_this());

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    delete layers_;
    layers_ = nullptr;

    tf_buffer_.reset();
    tf_listener_.reset();

    footprint_sub_.reset();
    costmap_sub_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  ~TestCollisionChecker() {}

  bool testPose(double x, double y, double theta)
  {
    geometry_msgs::msg::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;

    setPose(x, y, theta);
    publishFootprint();
    publishCostmap();
    return collision_checker_->isCollisionFree(pose);
  }

  void setFootprint(double footprint_padding, double robot_radius)
  {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    new_footprint = nav2_costmap_2d::makeFootprintFromRadius(robot_radius);
    nav2_costmap_2d::padFootprint(new_footprint, footprint_padding);
    footprint_ = new_footprint;
    layers_->setFootprint(footprint_);
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
    base_rel_map.header.stamp = now();
    tf_buffer_->setTransform(base_rel_map, "collision_checker_test");
  }

  void publishFootprint()
  {
    geometry_msgs::msg::PolygonStamped oriented_footprint;
    oriented_footprint.header.frame_id = global_frame_;
    oriented_footprint.header.stamp = now();
    nav2_costmap_2d::transformFootprint(x_, y_, yaw_, footprint_, oriented_footprint);
    footprint_sub_->setFootprint(
      std::make_shared<geometry_msgs::msg::PolygonStamped>(oriented_footprint));
  }

  void publishCostmap()
  {
    layers_->updateMap(x_, y_, yaw_);
    costmap_sub_->setCostmap(
      std::make_shared<nav2_msgs::msg::Costmap>(toCostmapMsg(layers_->getCostmap())));
  }

  nav2_msgs::msg::Costmap
  toCostmapMsg(nav2_costmap_2d::Costmap2D * costmap)
  {
    double resolution = costmap->getResolution();

    nav2_msgs::msg::Costmap costmap_msg;
    costmap_msg.header.frame_id = global_frame_;
    costmap_msg.header.stamp = now();

    costmap_msg.metadata.layer = "master";
    costmap_msg.metadata.resolution = resolution;

    costmap_msg.metadata.size_x = costmap->getSizeInCellsX();
    costmap_msg.metadata.size_y = costmap->getSizeInCellsY();

    double wx, wy;
    costmap->mapToWorld(0, 0, wx, wy);
    costmap_msg.metadata.origin.position.x = wx - resolution / 2;
    costmap_msg.metadata.origin.position.y = wy - resolution / 2;
    costmap_msg.metadata.origin.position.z = 0.0;
    costmap_msg.metadata.origin.orientation.w = 1.0;

    costmap_msg.data.resize(costmap_msg.metadata.size_x * costmap_msg.metadata.size_y);

    unsigned char * data = costmap->getCharMap();
    for (unsigned int i = 0; i < costmap_msg.data.size(); i++) {
      costmap_msg.data[i] = data[i];
    }

    return costmap_msg;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<DummyCostmapSubscriber> costmap_sub_;
  std::shared_ptr<DummyFootprintSubscriber> footprint_sub_;
  std::unique_ptr<nav2_costmap_2d::CollisionChecker> collision_checker_;
  std::string global_frame_;
  geometry_msgs::msg::TransformStamped base_rel_map;
  double x_, y_, yaw_;
  nav2_costmap_2d::LayeredCostmap * layers_{nullptr};
  std::vector<geometry_msgs::msg::Point> footprint_;
};


class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    collision_checker_ = std::make_shared<TestCollisionChecker>("test_collision_checker");
    collision_checker_->on_configure(collision_checker_->get_current_state());
    collision_checker_->on_activate(collision_checker_->get_current_state());
  }

  ~TestNode()
  {
    collision_checker_->on_deactivate(collision_checker_->get_current_state());
    collision_checker_->on_cleanup(collision_checker_->get_current_state());
  }

protected:
  std::shared_ptr<TestCollisionChecker> collision_checker_;
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
