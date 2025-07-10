// Copyright (c) 2025 Pranav Kolekar
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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "nav2_toolkit/pose_saver_node.hpp"

namespace fs = std::filesystem;

class PoseSaverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    unsigned int seed = static_cast<unsigned int>(
      std::chrono::system_clock::now().time_since_epoch().count());
    std::string unique_node_name = "pose_saver_node_" + std::to_string(rand_r(&seed));

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=" + unique_node_name});
    options.append_parameter_override("pose_file_path", test_pose_path_);
    options.append_parameter_override("auto_start_saving", true);
    options.append_parameter_override("auto_restore_pose", false);

    node_ = std::make_shared<nav2_toolkit::PoseSaverNode>(options);

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    spin_thread_ = std::make_unique<std::thread>([this]() {
      exec_->spin();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void TearDown() override
  {
    exec_->cancel();
    if (spin_thread_ && spin_thread_->joinable()) {
      spin_thread_->join();
    }

    exec_.reset();
    node_.reset();

    if (fs::exists(test_pose_path_)) {
      fs::remove(test_pose_path_);
    }
  }

  std::string test_pose_path_ = "/tmp/test_pose.yaml";
  std::shared_ptr<nav2_toolkit::PoseSaverNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::unique_ptr<std::thread> spin_thread_;
};

TEST_F(PoseSaverTest, test_set_last_pose_and_get_file_path)
{
  auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->pose.pose.position.x = 1.0;
  pose->pose.pose.position.y = 2.0;
  pose->pose.pose.orientation.w = 1.0;

  node_->test_set_last_pose(pose);
  EXPECT_EQ(node_->test_get_pose_file_path(), test_pose_path_);
}

TEST_F(PoseSaverTest, test_pose_file_write_and_read)
{
  auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->header.frame_id = "map";
  pose->pose.pose.position.x = 1.0;
  pose->pose.pose.position.y = 2.0;
  pose->pose.pose.orientation.w = 1.0;

  node_->test_set_last_pose(pose);
  node_->test_write_pose_to_file();

  auto result = node_->test_read_pose_from_file();

  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_NEAR(result.pose.pose.position.x, 1.0, 1e-5);
  EXPECT_NEAR(result.pose.pose.position.y, 2.0, 1e-5);
  EXPECT_NEAR(result.pose.pose.orientation.w, 1.0, 1e-5);
}

TEST_F(PoseSaverTest, test_start_pose_saver_service)
{
  auto client_node = std::make_shared<rclcpp::Node>("test_client_node");
  auto client = client_node->create_client<std_srvs::srv::Trigger>("start_pose_saver");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  auto status = rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(5));
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Pose saving started.");
}

TEST_F(PoseSaverTest, test_pose_written_by_timer_after_publish)
{
  auto pub_node = std::make_shared<rclcpp::Node>("test_pub_node");
  auto publisher = pub_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10);

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  msg.pose.pose.position.x = 3.14;
  msg.pose.pose.position.y = 2.71;
  msg.pose.pose.orientation.w = 1.0;

  rclcpp::Rate rate(10);
  for (int i = 0; i < 10; ++i) {
    publisher->publish(msg);
    rate.sleep();
  }

  std::this_thread::sleep_for(std::chrono::seconds(6));

  ASSERT_TRUE(fs::exists(test_pose_path_));

  YAML::Node node = YAML::LoadFile(test_pose_path_);
  double x = node["pose"]["position"]["x"].as<double>();
  double y = node["pose"]["position"]["y"].as<double>();
  double w = node["pose"]["orientation"]["w"].as<double>();

  EXPECT_NEAR(x, 3.14, 1e-5);
  EXPECT_NEAR(y, 2.71, 1e-5);
  EXPECT_NEAR(w, 1.0, 1e-5);
}

TEST_F(PoseSaverTest, test_stop_pose_saver_service)
{
  auto pub_node = std::make_shared<rclcpp::Node>("stop_test_pub_node");
  auto publisher = pub_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10);

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  msg.pose.pose.position.x = 5.55;
  msg.pose.pose.position.y = 4.44;
  msg.pose.pose.orientation.w = 1.0;

  rclcpp::Rate rate(10);
  for (int i = 0; i < 10; ++i) {
    publisher->publish(msg);
    rate.sleep();
  }

  std::this_thread::sleep_for(std::chrono::seconds(6));
  ASSERT_TRUE(fs::exists(test_pose_path_));

  auto client_node = std::make_shared<rclcpp::Node>("test_stop_client");
  auto stop_client = client_node->create_client<std_srvs::srv::Trigger>("stop_pose_saver");

  ASSERT_TRUE(stop_client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = stop_client->async_send_request(request);

  auto status = rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(5));
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Pose saving stopped.");

  fs::remove(test_pose_path_);
  ASSERT_FALSE(fs::exists(test_pose_path_));

  msg.pose.pose.position.x = 8.88;
  msg.pose.pose.position.y = 7.77;

  for (int i = 0; i < 10; ++i) {
    publisher->publish(msg);
    rate.sleep();
  }

  std::this_thread::sleep_for(std::chrono::seconds(6));

  EXPECT_FALSE(fs::exists(test_pose_path_));
}
