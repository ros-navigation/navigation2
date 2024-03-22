// Copyright (c) 2023 Smit Dumore
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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_loopback_sim/loopback_simulator.hpp"


class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class LoopbackSimulatorTest : public nav2_loopback_sim::LoopbackSimulator
{
public:
  LoopbackSimulatorTest()
  : nav2_loopback_sim::LoopbackSimulator(rclcpp::NodeOptions())
  {
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST(RobotSimulatorTest, TFUpdates) {
  // auto node = std::make_shared<LoopbackSimulatorTest>();
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(node);


  // node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  // node->tf_listener_ = std::make_shared
  //  <tf2_ros::TransformListener>(*(node->tf_buffer_));
  // node->tf_broadcaster_ = std::make_shared
  // <tf2_ros::TransformBroadcaster>(*(node));
  // node->cmd_vel_publisher_ = node->create_publisher
  // <geometry_msgs::msg::Twist>("cmd_vel", 10);
  // node->initial_pose_publisher_ = node->create_publisher
  //   <geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  // //std::this_thread::sleep_for(std::chrono::seconds(5));

  // geometry_msgs::msg::TransformStamped transform_stamped;
  // transform_stamped.header.stamp = node->now();
  // transform_stamped.header.frame_id = "map";
  // transform_stamped.child_frame_id = "odom";
  // transform_stamped.transform.translation.x = 5.0;
  // transform_stamped.transform.translation.y = 5.0;
  // transform_stamped.transform.translation.z = 0.0;
  // transform_stamped.transform.rotation.x = 0.0;
  // transform_stamped.transform.rotation.y = 0.0;
  // transform_stamped.transform.rotation.z = 0.0;
  // transform_stamped.transform.rotation.w = 1.0;
  // node->tf_broadcaster_->sendTransform(transform_stamped);

  // geometry_msgs::msg::TransformStamped transform_stamped;
  // transform_stamped.header.stamp = node->now();
  // transform_stamped.header.frame_id = "odom";
  // transform_stamped.child_frame_id = "base_link";
  // transform_stamped.transform.translation.x = 0.0;
  // transform_stamped.transform.translation.y = 0.0;
  // transform_stamped.transform.translation.z = 0.0;
  // transform_stamped.transform.rotation.x = 0.0;
  // transform_stamped.transform.rotation.y = 0.0;
  // transform_stamped.transform.rotation.z = 0.0;
  // transform_stamped.transform.rotation.w = 1.0;
  // node->tf_broadcaster_->sendTransform(transform_stamped);


  // Publish the initial pose
  // auto initial_pose_msg = std::make_shared
  // <geometry_msgs::msg::PoseWithCovarianceStamped>();

  // // Set the frame ID
  // initial_pose_msg->header.frame_id = "map";

  // // Set the timestamp
  // initial_pose_msg->header.stamp = node->now();

  // initial_pose_msg->pose.pose.position.x = 1.0;
  // initial_pose_msg->pose.pose.position.y = 1.0;
  // initial_pose_msg->pose.pose.position.z = 0.0;
  // initial_pose_msg->pose.pose.orientation.x = 0.0;
  // initial_pose_msg->pose.pose.orientation.y = 0.0;
  // initial_pose_msg->pose.pose.orientation.z = 0.0;
  // initial_pose_msg->pose.pose.orientation.w = 1.0;
  // node->initial_pose_publisher_->publish(*initial_pose_msg);


  // Publish cmd_vel messages
  // geometry_msgs::msg::Twist cmd_vel_msg;
  // cmd_vel_msg.linear.x = 0.5;
  // cmd_vel_msg.angular.z = 0.2;
  // std::cout << "publishing velo %\n";
  // node->cmd_vel_publisher_->publish(cmd_vel_msg);

  // try {
  //   geometry_msgs::msg::TransformStamped transformStamped =
  // node->tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

  //   EXPECT_NEAR(transformStamped.transform.translation.x, 1.0, 0.01);
  //   EXPECT_NEAR(transformStamped.transform.translation.y, 1.0, 0.01);
  // } catch (tf2::TransformException & ex) {
  //   EXPECT_TRUE(false) << "TF lookup failed: " << ex.what();
  // }

  // executor.spin();
  // executor.spin_some(std::chrono::seconds(10));
  // rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
