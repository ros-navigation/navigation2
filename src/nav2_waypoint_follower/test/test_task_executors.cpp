// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <math.h>
#include <condition_variable>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_waypoint_follower/plugins/photo_at_waypoint.hpp"
#include "nav2_waypoint_follower/plugins/wait_at_waypoint.hpp"
#include "nav2_waypoint_follower/plugins/input_at_waypoint.hpp"


class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WaypointFollowerTest, WaitAtWaypoint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testWaypointNode");

  node->declare_parameter("WAW.waypoint_pause_duration", 50);

  std::unique_ptr<nav2_waypoint_follower::WaitAtWaypoint> waw(
    new nav2_waypoint_follower::WaitAtWaypoint
  );
  waw->initialize(node, std::string("WAW"));

  auto start_time = node->now();

  // should wait 50ms
  geometry_msgs::msg::PoseStamped pose;
  waw->processAtWaypoint(pose, 0);

  auto end_time = node->now();

  EXPECT_NEAR((end_time - start_time).seconds(), 0.05, 0.01);

  waw.reset(new nav2_waypoint_follower::WaitAtWaypoint);
  node->set_parameter(rclcpp::Parameter("WAW.enabled", false));
  waw->initialize(node, std::string("WAW"));

  // plugin is not enabled, should exit
  EXPECT_TRUE(waw->processAtWaypoint(pose, 0));
}

TEST(WaypointFollowerTest, InputAtWaypoint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testWaypointNode");
  auto pub = node->create_publisher<std_msgs::msg::Empty>("input_at_waypoint/input", 1);
  pub->on_activate();
  auto publish_message =
    [&]() -> void
    {
      rclcpp::Rate(5).sleep();
      auto msg = std::make_unique<std_msgs::msg::Empty>();
      pub->publish(std::move(msg));
      rclcpp::spin_some(node->shared_from_this()->get_node_base_interface());
    };

  std::unique_ptr<nav2_waypoint_follower::InputAtWaypoint> iaw(
    new nav2_waypoint_follower::InputAtWaypoint
  );
  iaw->initialize(node, std::string("IAW"));

  auto start_time = node->now();

  // no input, should timeout
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_FALSE(iaw->processAtWaypoint(pose, 0));

  auto end_time = node->now();

  EXPECT_NEAR((end_time - start_time).seconds(), 10.0, 0.1);

  // has input now, should work
  std::thread t1(publish_message);
  EXPECT_TRUE(iaw->processAtWaypoint(pose, 0));
  t1.join();

  iaw.reset(new nav2_waypoint_follower::InputAtWaypoint);
  node->set_parameter(rclcpp::Parameter("IAW.enabled", false));
  iaw->initialize(node, std::string("IAW"));

  // plugin is not enabled, should exit
  EXPECT_TRUE(iaw->processAtWaypoint(pose, 0));
}

TEST(WaypointFollowerTest, PhotoAtWaypoint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testWaypointNode");
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 1);
  pub->on_activate();
  std::condition_variable cv;
  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx, std::defer_lock);
  bool data_published = false;
  auto publish_message =
    [&]() -> void
    {
      rclcpp::Rate(5).sleep();
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      // fill image msg data.
      msg->encoding = "rgb8";
      msg->height = 240;
      msg->width = 320;
      msg->step = 960;
      auto size = msg->height * msg->width * 3;
      msg->data.reserve(size);
      int fake_data = 0;
      for (size_t i = 0; i < size; i++) {
        msg->data.push_back(fake_data++);
      }
      pub->publish(std::move(msg));
      rclcpp::spin_some(node->shared_from_this()->get_node_base_interface());
      lck.lock();
      data_published = true;
      cv.notify_one();
      lck.unlock();
    };

  std::unique_ptr<nav2_waypoint_follower::PhotoAtWaypoint> paw(
    new nav2_waypoint_follower::PhotoAtWaypoint
  );
  paw->initialize(node, std::string("PAW"));

  // no images, throws because can't write
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_FALSE(paw->processAtWaypoint(pose, 0));

  std::thread t1(publish_message);
  cv.wait(lck);
  // has image now, since we force waiting until image is published
  EXPECT_TRUE(paw->processAtWaypoint(pose, 0));
  t1.join();

  paw.reset(new nav2_waypoint_follower::PhotoAtWaypoint);
  node->set_parameter(rclcpp::Parameter("PAW.enabled", false));
  paw->initialize(node, std::string("PAW"));

  // plugin is not enabled, should exit
  EXPECT_TRUE(paw->processAtWaypoint(pose, 0));
}
