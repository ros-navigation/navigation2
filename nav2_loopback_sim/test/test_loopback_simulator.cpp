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

class LoopbackSimulatorTest : public nav2_loopback_sim::LoopbackSimulator {
public:
    LoopbackSimulatorTest()
    : nav2_loopback_sim::LoopbackSimulator(rclcpp::NodeOptions())
    {    
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // // Initialize publisher for cmd_vel
        // cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // // Initilize a publisher for initial pose
        // initial_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //     "/initialpose", 10);

        // // Wait for the publisher to establish connections
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

TEST(RobotSimulatorTest, TFUpdates) {

    auto node = std::make_shared<LoopbackSimulatorTest>();

    node->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    node->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(node->tf_buffer_));
    node->cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    node->initial_pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);

    // Publish the initial pose
    auto initial_pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    initial_pose_msg->pose.pose.position.x = 1.0;
    initial_pose_msg->pose.pose.position.y = 1.0;
    initial_pose_msg->pose.pose.position.z = 0.0;
    initial_pose_msg->pose.pose.orientation.x = 0.0;
    initial_pose_msg->pose.pose.orientation.y = 0.0;
    initial_pose_msg->pose.pose.orientation.z = 0.0;
    initial_pose_msg->pose.pose.orientation.w = 1.0;
    node->initial_pose_publisher_->publish(*initial_pose_msg);


    // Publish cmd_vel messages
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.5;
    cmd_vel_msg.angular.z = 0.2;
    node->cmd_vel_publisher_->publish(cmd_vel_msg);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    try {
        node->tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        FAIL() << "TF lookup failed: " << ex.what();
    }
}

// TEST_F(RobotSimulatorTest, Relocalization) {
//     // Reset robot's pose to initial pose
//     // Send command to reset pose or directly manipulate TF tree
    
//     // Wait for the update to be processed
//     std::this_thread::sleep_for(std::chrono::seconds(1)); // Adjust sleep time as needed
    
//     // Check if the robot is correctly relocalized
//     // Assert conditions on the pose
//     // Example: ASSERT_NEAR(pose.x, initial_pose.x, tolerance);
// }

// int main(int argc, char **argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
