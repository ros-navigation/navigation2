// Copyright (c) 2022, Samsung Research America
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
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smoother/savitzky_golay_smoother.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace smoother_utils;  // NOLINT
using namespace nav2_smoother;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class SmootherWrapper : public nav2_smoother::SavitzkyGolaySmoother
{
public:
  SmootherWrapper()
  : nav2_smoother::SavitzkyGolaySmoother()
  {
  }
};

TEST(SmootherTest, test_sg_smoother)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSGSmootherTest");

  std::shared_ptr<nav2_msgs::msg::Costmap> costmap_msg =
    std::make_shared<nav2_msgs::msg::Costmap>();
  costmap_msg->header.stamp = node->now();
  costmap_msg->header.frame_id = "map";
  costmap_msg->data.resize(100 * 100);
  costmap_msg->metadata.resolution = 0.05;
  costmap_msg->metadata.size_x = 100;
  costmap_msg->metadata.size_y = 100;

  // island in the middle of lethal cost to cross
  for (unsigned int i = 20; i <= 30; ++i) {
    for (unsigned int j = 20; j <= 30; ++j) {
      costmap_msg->data[j * 100 + i] = 254;
    }
  }

  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> parent = node;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> dummy_costmap;
  dummy_costmap = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(parent, "dummy_topic");
  dummy_costmap->costmapCallback(costmap_msg);

  // Make smoother
  std::shared_ptr<tf2_ros::Buffer> dummy_tf;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> dummy_footprint;
  auto smoother = std::make_unique<SmootherWrapper>();
  smoother->configure(parent, "test", dummy_tf, dummy_costmap, dummy_footprint);
  rclcpp::Duration max_time = rclcpp::Duration::from_seconds(1.0);  // 1 seconds

  // Given nominal irregular path, test that the output is shorter and smoother

  // Test with refinement, even shorter and smoother


  // Test reversing / cusps segments



  // Test regular path, should see no effective change
  nav_msgs::msg::Path straight_regular_path, straight_regular_path_baseline;
  straight_regular_path.header.frame_id = "map";
  straight_regular_path.header.stamp = node->now();
  straight_regular_path.poses.resize(11);
  straight_regular_path.poses[0].pose.position.x = 0.5;
  straight_regular_path.poses[0].pose.position.y = 0.0;
  straight_regular_path.poses[1].pose.position.x = 0.5;
  straight_regular_path.poses[1].pose.position.y = 0.1;
  straight_regular_path.poses[2].pose.position.x = 0.5;
  straight_regular_path.poses[2].pose.position.y = 0.2;
  straight_regular_path.poses[3].pose.position.x = 0.5;
  straight_regular_path.poses[3].pose.position.y = 0.3;
  straight_regular_path.poses[4].pose.position.x = 0.5;
  straight_regular_path.poses[4].pose.position.y = 0.4;
  straight_regular_path.poses[5].pose.position.x = 0.5;
  straight_regular_path.poses[5].pose.position.y = 0.5;
  straight_regular_path.poses[6].pose.position.x = 0.5;
  straight_regular_path.poses[6].pose.position.y = 0.6;
  straight_regular_path.poses[7].pose.position.x = 0.5;
  straight_regular_path.poses[7].pose.position.y = 0.7;
  straight_regular_path.poses[8].pose.position.x = 0.5;
  straight_regular_path.poses[8].pose.position.y = 0.8;
  straight_regular_path.poses[9].pose.position.x = 0.5;
  straight_regular_path.poses[9].pose.position.y = 0.9;
  straight_regular_path.poses[10].pose.position.x = 0.5;
  straight_regular_path.poses[10].pose.position.y = 1.0;
  straight_regular_path_baseline = straight_regular_path;

  EXPECT_TRUE(smoother->smooth(straight_regular_path, max_time));
  for (uint i = 0; i != straight_regular_path.poses.size() - 1; i++) {
    // Check distances are still the same
    EXPECT_NEAR(
      fabs(
        straight_regular_path.poses[i].pose.position.y -
        straight_regular_path_baseline.poses[i].pose.position.y), 0.1, 0.001);
  }

  // Attempt smoothing with no time given
  rclcpp::Duration no_time = rclcpp::Duration::from_seconds(0.0);  // 0 seconds
  EXPECT_FALSE(smoother->smooth(straight_regular_path, no_time));
}
