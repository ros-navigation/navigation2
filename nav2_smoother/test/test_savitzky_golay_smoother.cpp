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
#include <random>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/smoother_exceptions.hpp"
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

TEST(SmootherTest, test_sg_smoother_basics)
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

  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> parent = node;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> dummy_costmap;
  dummy_costmap = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(parent, "dummy_topic");
  dummy_costmap->costmapCallback(costmap_msg);

  // Make smoother
  std::shared_ptr<tf2_ros::Buffer> dummy_tf;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> dummy_footprint;
  node->declare_parameter("test.do_refinement", rclcpp::ParameterValue(false));
  auto smoother = std::make_unique<nav2_smoother::SavitzkyGolaySmoother>();
  smoother->configure(parent, "test", dummy_tf, dummy_costmap, dummy_footprint);
  smoother->activate();
  rclcpp::Duration max_time = rclcpp::Duration::from_seconds(1.0);  // 1 seconds

  // Test regular path, should see no effective change
  nav_msgs::msg::Path straight_regular_path, straight_regular_path_baseline;
  straight_regular_path.header.frame_id = "map";
  straight_regular_path.header.stamp = node->now();
  straight_regular_path.poses.resize(11);
  straight_regular_path.poses[0].pose.position.x = 0.5;
  straight_regular_path.poses[0].pose.position.y = 0.1;
  straight_regular_path.poses[1].pose.position.x = 0.5;
  straight_regular_path.poses[1].pose.position.y = 0.2;
  straight_regular_path.poses[2].pose.position.x = 0.5;
  straight_regular_path.poses[2].pose.position.y = 0.3;
  straight_regular_path.poses[3].pose.position.x = 0.5;
  straight_regular_path.poses[3].pose.position.y = 0.4;
  straight_regular_path.poses[4].pose.position.x = 0.5;
  straight_regular_path.poses[4].pose.position.y = 0.5;
  straight_regular_path.poses[5].pose.position.x = 0.5;
  straight_regular_path.poses[5].pose.position.y = 0.6;
  straight_regular_path.poses[6].pose.position.x = 0.5;
  straight_regular_path.poses[6].pose.position.y = 0.7;
  straight_regular_path.poses[7].pose.position.x = 0.5;
  straight_regular_path.poses[7].pose.position.y = 0.8;
  straight_regular_path.poses[8].pose.position.x = 0.5;
  straight_regular_path.poses[8].pose.position.y = 0.9;
  straight_regular_path.poses[9].pose.position.x = 0.5;
  straight_regular_path.poses[9].pose.position.y = 1.0;
  straight_regular_path.poses[10].pose.position.x = 0.5;
  straight_regular_path.poses[10].pose.position.y = 1.1;
  straight_regular_path_baseline = straight_regular_path;

  EXPECT_TRUE(smoother->smooth(straight_regular_path, max_time));
  for (uint i = 0; i != straight_regular_path.poses.size() - 1; i++) {
    // Check distances are still the same
    EXPECT_NEAR(
      fabs(
        straight_regular_path.poses[i].pose.position.y -
        straight_regular_path_baseline.poses[i].pose.position.y), 0.0, 0.011);
  }

  // Attempt smoothing with no time given, should fail
  rclcpp::Duration no_time = rclcpp::Duration::from_seconds(-1.0);  // 0 seconds
  EXPECT_THROW(smoother->smooth(straight_regular_path, no_time), nav2_core::SmootherTimedOut);

  smoother->deactivate();
  smoother->cleanup();
}

TEST(SmootherTest, test_sg_smoother_noisey_path)
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

  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> parent = node;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> dummy_costmap;
  dummy_costmap = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(parent, "dummy_topic");
  dummy_costmap->costmapCallback(costmap_msg);

  // Make smoother
  std::shared_ptr<tf2_ros::Buffer> dummy_tf;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> dummy_footprint;
  node->declare_parameter("test.do_refinement", rclcpp::ParameterValue(false));
  auto smoother = std::make_unique<nav2_smoother::SavitzkyGolaySmoother>();
  smoother->configure(parent, "test", dummy_tf, dummy_costmap, dummy_footprint);
  rclcpp::Duration max_time = rclcpp::Duration::from_seconds(1.0);  // 1 seconds

  // Given nominal irregular/noisey path, test that the output is shorter and smoother
  nav_msgs::msg::Path noisey_path, noisey_path_baseline;
  noisey_path.header.frame_id = "map";
  noisey_path.header.stamp = node->now();
  noisey_path.poses.resize(11);
  noisey_path.poses[0].pose.position.x = 0.5;
  noisey_path.poses[0].pose.position.y = 0.1;
  noisey_path.poses[1].pose.position.x = 0.5;
  noisey_path.poses[1].pose.position.y = 0.2;
  noisey_path.poses[2].pose.position.x = 0.5;
  noisey_path.poses[2].pose.position.y = 0.3;
  noisey_path.poses[3].pose.position.x = 0.5;
  noisey_path.poses[3].pose.position.y = 0.4;
  noisey_path.poses[4].pose.position.x = 0.5;
  noisey_path.poses[4].pose.position.y = 0.5;
  noisey_path.poses[5].pose.position.x = 0.5;
  noisey_path.poses[5].pose.position.y = 0.6;
  noisey_path.poses[6].pose.position.x = 0.5;
  noisey_path.poses[6].pose.position.y = 0.7;
  noisey_path.poses[7].pose.position.x = 0.5;
  noisey_path.poses[7].pose.position.y = 0.8;
  noisey_path.poses[8].pose.position.x = 0.5;
  noisey_path.poses[8].pose.position.y = 0.9;
  noisey_path.poses[9].pose.position.x = 0.5;
  noisey_path.poses[9].pose.position.y = 1.0;
  noisey_path.poses[10].pose.position.x = 0.5;
  noisey_path.poses[10].pose.position.y = 1.1;

  // Add random but deterministic noises
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> normal_distribution{0.0, 0.02};
  for (unsigned int i = 0; i != noisey_path.poses.size(); i++) {
    auto noise = normal_distribution(gen);
    noisey_path.poses[i].pose.position.x += noise;
  }

  noisey_path_baseline = noisey_path;
  EXPECT_TRUE(smoother->smooth(noisey_path, max_time));

  // Compute metric, should be shorter if smoother
  double length = 0;
  double base_length = 0;
  for (unsigned int i = 0; i != noisey_path.poses.size() - 1; i++) {
    length += std::hypot(
      noisey_path.poses[i + 1].pose.position.x - noisey_path.poses[i].pose.position.x,
      noisey_path.poses[i + 1].pose.position.y - noisey_path.poses[i].pose.position.y);
    base_length += std::hypot(
      noisey_path_baseline.poses[i + 1].pose.position.x -
      noisey_path_baseline.poses[i].pose.position.x,
      noisey_path_baseline.poses[i + 1].pose.position.y -
      noisey_path_baseline.poses[i].pose.position.y);
  }

  EXPECT_LT(length, base_length);

  // Test again with refinement, even shorter and smoother
  node->set_parameter(rclcpp::Parameter("test.do_refinement", rclcpp::ParameterValue(true)));
  smoother->configure(parent, "test", dummy_tf, dummy_costmap, dummy_footprint);
  nav_msgs::msg::Path noisey_path_refined = noisey_path_baseline;
  EXPECT_TRUE(smoother->smooth(noisey_path_refined, max_time));

  length = 0;
  for (unsigned int i = 0; i != noisey_path.poses.size() - 1; i++) {
    length += std::hypot(
      noisey_path_refined.poses[i + 1].pose.position.x -
      noisey_path_refined.poses[i].pose.position.x,
      noisey_path_refined.poses[i + 1].pose.position.y -
      noisey_path_refined.poses[i].pose.position.y);
  }

  EXPECT_LT(length, base_length);
}

TEST(SmootherTest, test_sg_smoother_reversing)
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

  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> parent = node;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> dummy_costmap;
  dummy_costmap = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(parent, "dummy_topic");
  dummy_costmap->costmapCallback(costmap_msg);

  // Make smoother
  std::shared_ptr<tf2_ros::Buffer> dummy_tf;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> dummy_footprint;
  node->declare_parameter("test.do_refinement", rclcpp::ParameterValue(false));
  auto smoother = std::make_unique<nav2_smoother::SavitzkyGolaySmoother>();
  smoother->configure(parent, "test", dummy_tf, dummy_costmap, dummy_footprint);
  rclcpp::Duration max_time = rclcpp::Duration::from_seconds(1.0);  // 1 seconds

  // Test reversing / multiple segments via a cusp
  nav_msgs::msg::Path cusp_path, cusp_path_baseline;
  cusp_path.header.frame_id = "map";
  cusp_path.header.stamp = node->now();
  cusp_path.poses.resize(22);
  cusp_path.poses[0].pose.position.x = 0.5;
  cusp_path.poses[0].pose.position.y = 0.1;
  cusp_path.poses[1].pose.position.x = 0.5;
  cusp_path.poses[1].pose.position.y = 0.2;
  cusp_path.poses[2].pose.position.x = 0.5;
  cusp_path.poses[2].pose.position.y = 0.3;
  cusp_path.poses[3].pose.position.x = 0.5;
  cusp_path.poses[3].pose.position.y = 0.4;
  cusp_path.poses[4].pose.position.x = 0.5;
  cusp_path.poses[4].pose.position.y = 0.5;
  cusp_path.poses[5].pose.position.x = 0.5;
  cusp_path.poses[5].pose.position.y = 0.6;
  cusp_path.poses[6].pose.position.x = 0.5;
  cusp_path.poses[6].pose.position.y = 0.7;
  cusp_path.poses[7].pose.position.x = 0.5;
  cusp_path.poses[7].pose.position.y = 0.8;
  cusp_path.poses[8].pose.position.x = 0.5;
  cusp_path.poses[8].pose.position.y = 0.9;
  cusp_path.poses[9].pose.position.x = 0.5;
  cusp_path.poses[9].pose.position.y = 1.0;
  cusp_path.poses[10].pose.position.x = 0.5;
  cusp_path.poses[10].pose.position.y = 1.1;
  cusp_path.poses[11].pose.position.x = 0.5;
  cusp_path.poses[11].pose.position.y = 1.0;
  cusp_path.poses[12].pose.position.x = 0.5;
  cusp_path.poses[12].pose.position.y = 0.9;
  cusp_path.poses[13].pose.position.x = 0.5;
  cusp_path.poses[13].pose.position.y = 0.8;
  cusp_path.poses[14].pose.position.x = 0.5;
  cusp_path.poses[14].pose.position.y = 0.7;
  cusp_path.poses[15].pose.position.x = 0.5;
  cusp_path.poses[15].pose.position.y = 0.6;
  cusp_path.poses[16].pose.position.x = 0.5;
  cusp_path.poses[16].pose.position.y = 0.5;
  cusp_path.poses[17].pose.position.x = 0.5;
  cusp_path.poses[17].pose.position.y = 0.4;
  cusp_path.poses[18].pose.position.x = 0.5;
  cusp_path.poses[18].pose.position.y = 0.3;
  cusp_path.poses[19].pose.position.x = 0.5;
  cusp_path.poses[19].pose.position.y = 0.2;
  cusp_path.poses[20].pose.position.x = 0.5;
  cusp_path.poses[20].pose.position.y = 0.1;
  cusp_path.poses[21].pose.position.x = 0.5;
  cusp_path.poses[21].pose.position.y = 0.0;

  // Add random but deterministic noises
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> normal_distribution{0.0, 0.02};
  for (unsigned int i = 0; i != cusp_path.poses.size(); i++) {
    auto noise = normal_distribution(gen);
    cusp_path.poses[i].pose.position.x += noise;
  }

  cusp_path_baseline = cusp_path;

  EXPECT_TRUE(smoother->smooth(cusp_path, max_time));

  // If it detected the cusp, the cusp point should be fixed
  EXPECT_EQ(cusp_path.poses[10].pose.position.x, cusp_path_baseline.poses[10].pose.position.x);
  EXPECT_EQ(cusp_path.poses[10].pose.position.y, cusp_path_baseline.poses[10].pose.position.y);

  // But the path also should be smoother / shorter
  double length = 0;
  double base_length = 0;
  for (unsigned int i = 0; i != cusp_path.poses.size() - 1; i++) {
    length += std::hypot(
      cusp_path.poses[i + 1].pose.position.x - cusp_path.poses[i].pose.position.x,
      cusp_path.poses[i + 1].pose.position.y - cusp_path.poses[i].pose.position.y);
    base_length += std::hypot(
      cusp_path_baseline.poses[i + 1].pose.position.x -
      cusp_path_baseline.poses[i].pose.position.x,
      cusp_path_baseline.poses[i + 1].pose.position.y -
      cusp_path_baseline.poses[i].pose.position.y);
  }

  EXPECT_LT(length, base_length);
}
