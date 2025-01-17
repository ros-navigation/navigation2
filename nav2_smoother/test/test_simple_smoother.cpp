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
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_smoother/simple_smoother.hpp"
#include "nav2_core/smoother_exceptions.hpp"

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

class SmootherWrapper : public nav2_smoother::SimpleSmoother
{
public:
  SmootherWrapper()
  : nav2_smoother::SimpleSmoother()
  {
  }

  std::vector<PathSegment> findDirectionalPathSegmentsWrapper(nav_msgs::msg::Path path)
  {
    return findDirectionalPathSegments(path);
  }

  void setMaxItsToInvalid()
  {
    max_its_ = 0;
  }
};

TEST(SmootherTest, test_simple_smoother)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSmootherTest");

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

  // Test that an irregular distributed path becomes more distributed
  nav_msgs::msg::Path straight_irregular_path;
  straight_irregular_path.header.frame_id = "map";
  straight_irregular_path.header.stamp = node->now();
  straight_irregular_path.poses.resize(11);
  straight_irregular_path.poses[0].pose.position.x = 0.5;
  straight_irregular_path.poses[0].pose.position.y = 0.0;
  straight_irregular_path.poses[1].pose.position.x = 0.5;
  straight_irregular_path.poses[1].pose.position.y = 0.1;
  straight_irregular_path.poses[2].pose.position.x = 0.5;
  straight_irregular_path.poses[2].pose.position.y = 0.2;
  straight_irregular_path.poses[3].pose.position.x = 0.5;
  straight_irregular_path.poses[3].pose.position.y = 0.35;
  straight_irregular_path.poses[4].pose.position.x = 0.5;
  straight_irregular_path.poses[4].pose.position.y = 0.4;
  straight_irregular_path.poses[5].pose.position.x = 0.5;
  straight_irregular_path.poses[5].pose.position.y = 0.56;
  straight_irregular_path.poses[6].pose.position.x = 0.5;
  straight_irregular_path.poses[6].pose.position.y = 0.9;
  straight_irregular_path.poses[7].pose.position.x = 0.5;
  straight_irregular_path.poses[7].pose.position.y = 0.95;
  straight_irregular_path.poses[8].pose.position.x = 0.5;
  straight_irregular_path.poses[8].pose.position.y = 1.3;
  straight_irregular_path.poses[9].pose.position.x = 0.5;
  straight_irregular_path.poses[9].pose.position.y = 2.0;
  straight_irregular_path.poses[10].pose.position.x = 0.5;
  straight_irregular_path.poses[10].pose.position.y = 2.5;

  rclcpp::Duration no_time = rclcpp::Duration::from_seconds(0.0);  // 0 seconds
  rclcpp::Duration max_time = rclcpp::Duration::from_seconds(1);  // 1 second
  EXPECT_THROW(smoother->smooth(straight_irregular_path, no_time), nav2_core::SmootherTimedOut);
  EXPECT_TRUE(smoother->smooth(straight_irregular_path, max_time));
  for (uint i = 0; i != straight_irregular_path.poses.size() - 1; i++) {
    // Check distances are more evenly spaced out now
    EXPECT_LT(
      fabs(
        straight_irregular_path.poses[i].pose.position.y -
        straight_irregular_path.poses[i + 1].pose.position.y), 0.50);
  }

  // Test regular path, should see no effective change
  nav_msgs::msg::Path straight_regular_path;
  straight_regular_path.header = straight_irregular_path.header;
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

  EXPECT_TRUE(smoother->smooth(straight_regular_path, max_time));
  for (uint i = 0; i != straight_regular_path.poses.size() - 1; i++) {
    // Check distances are still very evenly spaced
    EXPECT_NEAR(
      fabs(
        straight_regular_path.poses[i].pose.position.y -
        straight_regular_path.poses[i + 1].pose.position.y), 0.1, 0.001);
  }

  // test shorter and curved if given a right angle
  nav_msgs::msg::Path right_angle_path;
  right_angle_path = straight_regular_path;
  straight_regular_path.poses[6].pose.position.x = 0.6;
  straight_regular_path.poses[6].pose.position.y = 0.5;
  straight_regular_path.poses[7].pose.position.x = 0.7;
  straight_regular_path.poses[7].pose.position.y = 0.5;
  straight_regular_path.poses[8].pose.position.x = 0.8;
  straight_regular_path.poses[8].pose.position.y = 0.5;
  straight_regular_path.poses[9].pose.position.x = 0.9;
  straight_regular_path.poses[9].pose.position.y = 0.5;
  straight_regular_path.poses[10].pose.position.x = 0.95;
  straight_regular_path.poses[10].pose.position.y = 0.5;
  EXPECT_TRUE(smoother->smooth(straight_regular_path, max_time));
  EXPECT_NEAR(straight_regular_path.poses[5].pose.position.x, 0.607, 0.01);
  EXPECT_NEAR(straight_regular_path.poses[5].pose.position.y, 0.387, 0.01);

  // Test that collisions are disregarded
  nav_msgs::msg::Path collision_path;
  collision_path.poses.resize(11);
  collision_path.poses[0].pose.position.x = 0.0;
  collision_path.poses[0].pose.position.y = 0.0;
  collision_path.poses[1].pose.position.x = 0.2;
  collision_path.poses[1].pose.position.y = 0.2;
  collision_path.poses[2].pose.position.x = 0.4;
  collision_path.poses[2].pose.position.y = 0.4;
  collision_path.poses[3].pose.position.x = 0.6;
  collision_path.poses[3].pose.position.y = 0.6;
  collision_path.poses[4].pose.position.x = 0.8;
  collision_path.poses[4].pose.position.y = 0.8;
  collision_path.poses[5].pose.position.x = 1.0;
  collision_path.poses[5].pose.position.y = 1.0;
  collision_path.poses[6].pose.position.x = 1.1;
  collision_path.poses[6].pose.position.y = 1.1;
  collision_path.poses[7].pose.position.x = 1.2;
  collision_path.poses[7].pose.position.y = 1.2;
  collision_path.poses[8].pose.position.x = 1.3;
  collision_path.poses[8].pose.position.y = 1.3;
  collision_path.poses[9].pose.position.x = 1.4;
  collision_path.poses[9].pose.position.y = 1.4;
  collision_path.poses[10].pose.position.x = 1.5;
  collision_path.poses[10].pose.position.y = 1.5;
  EXPECT_TRUE(smoother->smooth(collision_path, max_time));

  // test cusp / reversing segments
  nav_msgs::msg::Path reversing_path;
  reversing_path.poses.resize(11);
  reversing_path.poses[0].pose.position.x = 0.5;
  reversing_path.poses[0].pose.position.y = 0.0;
  reversing_path.poses[1].pose.position.x = 0.5;
  reversing_path.poses[1].pose.position.y = 0.1;
  reversing_path.poses[2].pose.position.x = 0.5;
  reversing_path.poses[2].pose.position.y = 0.2;
  reversing_path.poses[3].pose.position.x = 0.5;
  reversing_path.poses[3].pose.position.y = 0.3;
  reversing_path.poses[4].pose.position.x = 0.5;
  reversing_path.poses[4].pose.position.y = 0.4;
  reversing_path.poses[5].pose.position.x = 0.5;
  reversing_path.poses[5].pose.position.y = 0.5;
  reversing_path.poses[6].pose.position.x = 0.5;
  reversing_path.poses[6].pose.position.y = 0.4;
  reversing_path.poses[7].pose.position.x = 0.5;
  reversing_path.poses[7].pose.position.y = 0.3;
  reversing_path.poses[8].pose.position.x = 0.5;
  reversing_path.poses[8].pose.position.y = 0.2;
  reversing_path.poses[9].pose.position.x = 0.5;
  reversing_path.poses[9].pose.position.y = 0.1;
  reversing_path.poses[10].pose.position.x = 0.5;
  reversing_path.poses[10].pose.position.y = 0.0;
  EXPECT_TRUE(smoother->smooth(reversing_path, max_time));

  // test rotate in place
  tf2::Quaternion quat1, quat2;
  quat1.setRPY(0.0, 0.0, 0.0);
  quat2.setRPY(0.0, 0.0, 1.0);
  straight_irregular_path.poses[5].pose.position.x = 0.5;
  straight_irregular_path.poses[5].pose.position.y = 0.5;
  straight_irregular_path.poses[5].pose.orientation = tf2::toMsg(quat1);
  straight_irregular_path.poses[6].pose.position.x = 0.5;
  straight_irregular_path.poses[6].pose.position.y = 0.5;
  straight_irregular_path.poses[6].pose.orientation = tf2::toMsg(quat2);
  EXPECT_TRUE(smoother->smooth(straight_irregular_path, max_time));

  // test approach
  nav_msgs::msg::Path approach_path;
  approach_path.poses.resize(3);
  approach_path.poses[0].pose.position.x = 0.5;
  approach_path.poses[0].pose.position.y = 0.0;
  approach_path.poses[1].pose.position.x = 0.5;
  approach_path.poses[1].pose.position.y = 0.1;
  approach_path.poses[2].pose.position.x = 0.5;
  approach_path.poses[2].pose.position.y = 0.2;
  EXPECT_TRUE(smoother->smooth(approach_path, max_time));

  // test max iterations
  smoother->setMaxItsToInvalid();
  nav_msgs::msg::Path max_its_path;
  max_its_path.poses.resize(11);
  max_its_path.poses[0].pose.position.x = 0.5;
  max_its_path.poses[0].pose.position.y = 0.0;
  max_its_path.poses[1].pose.position.x = 0.5;
  max_its_path.poses[1].pose.position.y = 0.1;
  max_its_path.poses[2].pose.position.x = 0.5;
  max_its_path.poses[2].pose.position.y = 0.2;
  max_its_path.poses[3].pose.position.x = 0.5;
  max_its_path.poses[3].pose.position.y = 0.3;
  max_its_path.poses[4].pose.position.x = 0.5;
  max_its_path.poses[4].pose.position.y = 0.4;
  max_its_path.poses[5].pose.position.x = 0.5;
  max_its_path.poses[5].pose.position.y = 0.5;
  max_its_path.poses[6].pose.position.x = 0.5;
  max_its_path.poses[6].pose.position.y = 0.6;
  max_its_path.poses[7].pose.position.x = 0.5;
  max_its_path.poses[7].pose.position.y = 0.7;
  max_its_path.poses[8].pose.position.x = 0.5;
  max_its_path.poses[8].pose.position.y = 0.8;
  max_its_path.poses[9].pose.position.x = 0.5;
  max_its_path.poses[9].pose.position.y = 0.9;
  max_its_path.poses[10].pose.position.x = 0.5;
  max_its_path.poses[10].pose.position.y = 1.0;
  EXPECT_TRUE(smoother->smooth(max_its_path, max_time));
}
