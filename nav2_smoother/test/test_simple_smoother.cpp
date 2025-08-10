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
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_smoother/simple_smoother.hpp"
#include "nav2_core/smoother_exceptions.hpp"

using namespace smoother_utils;  // NOLINT
using namespace nav2_smoother;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

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

  void setMaxIts(const int max_its)
  {
    max_its_ = max_its;
  }

  void setIsHolonomic(const bool is_holonomic)
  {
    is_holonomic_ = is_holonomic;
  }

  void setMinimumTurningRadius(const double min_turning_radius)
  {
    min_turning_radius_ = min_turning_radius;
  }

  double getPathLength(const nav_msgs::msg::Path & path)
  {
    if (path.poses.size() == 0) {
      return 0.0;
    }

    double path_length = 0.0;
    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
      double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      path_length += std::hypot(dx, dy);
    }
    return path_length;
  }
};

TEST(SmootherTest, test_simple_smoother)
{
  nav2::LifecycleNode::SharedPtr node =
    std::make_shared<nav2::LifecycleNode>("SmacSmootherTest");

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

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> dummy_costmap;
  dummy_costmap = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node, "dummy_topic");
  dummy_costmap->costmapCallback(costmap_msg);

  // Make smoother
  std::shared_ptr<tf2_ros::Buffer> dummy_tf;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> dummy_footprint;
  auto smoother = std::make_unique<SmootherWrapper>();
  smoother->configure(node, "test", dummy_tf, dummy_costmap, dummy_footprint);

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
  EXPECT_FALSE(smoother->smooth(collision_path, max_time));

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
  smoother->setMaxIts(0.0);
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
  EXPECT_FALSE(smoother->smooth(max_its_path, max_time));

  // test nonholonomic
  smoother->setIsHolonomic(false);
  smoother->setMinimumTurningRadius(0.4);
  smoother->setMaxIts(1000);
  nav_msgs::msg::Path smac_path;
  smac_path.poses.resize(37);
  smac_path.poses[0].pose.position.x = 0.250000;
  smac_path.poses[0].pose.position.y = 0.250000;
  smac_path.poses[0].pose.orientation.x = 0.000000;
  smac_path.poses[0].pose.orientation.y = 0.000000;
  smac_path.poses[0].pose.orientation.z = 0.000000;
  smac_path.poses[0].pose.orientation.w = 1.000000;
  smac_path.poses[1].pose.position.x = 0.353528;
  smac_path.poses[1].pose.position.y = 0.263630;
  smac_path.poses[1].pose.orientation.x = 0.000000;
  smac_path.poses[1].pose.orientation.y = 0.000000;
  smac_path.poses[1].pose.orientation.z = 0.130526;
  smac_path.poses[1].pose.orientation.w = 0.991445;
  smac_path.poses[2].pose.position.x = 0.450000;
  smac_path.poses[2].pose.position.y = 0.303590;
  smac_path.poses[2].pose.orientation.x = 0.000000;
  smac_path.poses[2].pose.orientation.y = 0.000000;
  smac_path.poses[2].pose.orientation.z = 0.258819;
  smac_path.poses[2].pose.orientation.w = 0.965926;
  smac_path.poses[3].pose.position.x = 0.540431;
  smac_path.poses[3].pose.position.y = 0.355800;
  smac_path.poses[3].pose.orientation.x = 0.000000;
  smac_path.poses[3].pose.orientation.y = 0.000000;
  smac_path.poses[3].pose.orientation.z = 0.258819;
  smac_path.poses[3].pose.orientation.w = 0.965926;
  smac_path.poses[4].pose.position.x = 0.630862;
  smac_path.poses[4].pose.position.y = 0.408011;
  smac_path.poses[4].pose.orientation.x = 0.000000;
  smac_path.poses[4].pose.orientation.y = 0.000000;
  smac_path.poses[4].pose.orientation.z = 0.258819;
  smac_path.poses[4].pose.orientation.w = 0.965926;
  smac_path.poses[5].pose.position.x = 0.721294;
  smac_path.poses[5].pose.position.y = 0.460221;
  smac_path.poses[5].pose.orientation.x = 0.000000;
  smac_path.poses[5].pose.orientation.y = 0.000000;
  smac_path.poses[5].pose.orientation.z = 0.258819;
  smac_path.poses[5].pose.orientation.w = 0.965926;
  smac_path.poses[6].pose.position.x = 0.811725;
  smac_path.poses[6].pose.position.y = 0.512432;
  smac_path.poses[6].pose.orientation.x = 0.000000;
  smac_path.poses[6].pose.orientation.y = 0.000000;
  smac_path.poses[6].pose.orientation.z = 0.258819;
  smac_path.poses[6].pose.orientation.w = 0.965926;
  smac_path.poses[7].pose.position.x = 0.902156;
  smac_path.poses[7].pose.position.y = 0.564642;
  smac_path.poses[7].pose.orientation.x = 0.000000;
  smac_path.poses[7].pose.orientation.y = 0.000000;
  smac_path.poses[7].pose.orientation.z = 0.258819;
  smac_path.poses[7].pose.orientation.w = 0.965926;
  smac_path.poses[8].pose.position.x = 0.992587;
  smac_path.poses[8].pose.position.y = 0.616853;
  smac_path.poses[8].pose.orientation.x = 0.000000;
  smac_path.poses[8].pose.orientation.y = 0.000000;
  smac_path.poses[8].pose.orientation.z = 0.258819;
  smac_path.poses[8].pose.orientation.w = 0.965926;
  smac_path.poses[9].pose.position.x = 1.083018;
  smac_path.poses[9].pose.position.y = 0.669063;
  smac_path.poses[9].pose.orientation.x = 0.000000;
  smac_path.poses[9].pose.orientation.y = 0.000000;
  smac_path.poses[9].pose.orientation.z = 0.258819;
  smac_path.poses[9].pose.orientation.w = 0.965926;
  smac_path.poses[10].pose.position.x = 1.173450;
  smac_path.poses[10].pose.position.y = 0.721274;
  smac_path.poses[10].pose.orientation.x = 0.000000;
  smac_path.poses[10].pose.orientation.y = 0.000000;
  smac_path.poses[10].pose.orientation.z = 0.258819;
  smac_path.poses[10].pose.orientation.w = 0.965926;
  smac_path.poses[11].pose.position.x = 1.263881;
  smac_path.poses[11].pose.position.y = 0.773484;
  smac_path.poses[11].pose.orientation.x = 0.000000;
  smac_path.poses[11].pose.orientation.y = 0.000000;
  smac_path.poses[11].pose.orientation.z = 0.258819;
  smac_path.poses[11].pose.orientation.w = 0.965926;
  smac_path.poses[12].pose.position.x = 1.354312;
  smac_path.poses[12].pose.position.y = 0.825695;
  smac_path.poses[12].pose.orientation.x = 0.000000;
  smac_path.poses[12].pose.orientation.y = 0.000000;
  smac_path.poses[12].pose.orientation.z = 0.258819;
  smac_path.poses[12].pose.orientation.w = 0.965926;
  smac_path.poses[13].pose.position.x = 1.437155;
  smac_path.poses[13].pose.position.y = 0.889262;
  smac_path.poses[13].pose.orientation.x = 0.000000;
  smac_path.poses[13].pose.orientation.y = 0.000000;
  smac_path.poses[13].pose.orientation.z = 0.382683;
  smac_path.poses[13].pose.orientation.w = 0.923880;
  smac_path.poses[14].pose.position.x = 1.510992;
  smac_path.poses[14].pose.position.y = 0.963099;
  smac_path.poses[14].pose.orientation.x = 0.000000;
  smac_path.poses[14].pose.orientation.y = 0.000000;
  smac_path.poses[14].pose.orientation.z = 0.382683;
  smac_path.poses[14].pose.orientation.w = 0.923880;
  smac_path.poses[15].pose.position.x = 1.584828;
  smac_path.poses[15].pose.position.y = 1.036936;
  smac_path.poses[15].pose.orientation.x = 0.000000;
  smac_path.poses[15].pose.orientation.y = 0.000000;
  smac_path.poses[15].pose.orientation.z = 0.382683;
  smac_path.poses[15].pose.orientation.w = 0.923880;
  smac_path.poses[16].pose.position.x = 1.658665;
  smac_path.poses[16].pose.position.y = 1.110772;
  smac_path.poses[16].pose.orientation.x = 0.000000;
  smac_path.poses[16].pose.orientation.y = 0.000000;
  smac_path.poses[16].pose.orientation.z = 0.382683;
  smac_path.poses[16].pose.orientation.w = 0.923880;
  smac_path.poses[17].pose.position.x = 1.732502;
  smac_path.poses[17].pose.position.y = 1.184609;
  smac_path.poses[17].pose.orientation.x = 0.000000;
  smac_path.poses[17].pose.orientation.y = 0.000000;
  smac_path.poses[17].pose.orientation.z = 0.382683;
  smac_path.poses[17].pose.orientation.w = 0.923880;
  smac_path.poses[18].pose.position.x = 1.806339;
  smac_path.poses[18].pose.position.y = 1.258446;
  smac_path.poses[18].pose.orientation.x = 0.000000;
  smac_path.poses[18].pose.orientation.y = 0.000000;
  smac_path.poses[18].pose.orientation.z = 0.382683;
  smac_path.poses[18].pose.orientation.w = 0.923880;
  smac_path.poses[19].pose.position.x = 1.880175;
  smac_path.poses[19].pose.position.y = 1.332283;
  smac_path.poses[19].pose.orientation.x = 0.000000;
  smac_path.poses[19].pose.orientation.y = 0.000000;
  smac_path.poses[19].pose.orientation.z = 0.382683;
  smac_path.poses[19].pose.orientation.w = 0.923880;
  smac_path.poses[20].pose.position.x = 1.943743;
  smac_path.poses[20].pose.position.y = 1.415126;
  smac_path.poses[20].pose.orientation.x = 0.000000;
  smac_path.poses[20].pose.orientation.y = 0.000000;
  smac_path.poses[20].pose.orientation.z = 0.500000;
  smac_path.poses[20].pose.orientation.w = 0.866025;
  smac_path.poses[21].pose.position.x = 1.995953;
  smac_path.poses[21].pose.position.y = 1.505557;
  smac_path.poses[21].pose.orientation.x = 0.000000;
  smac_path.poses[21].pose.orientation.y = 0.000000;
  smac_path.poses[21].pose.orientation.z = 0.500000;
  smac_path.poses[21].pose.orientation.w = 0.866025;
  smac_path.poses[22].pose.position.x = 2.035913;
  smac_path.poses[22].pose.position.y = 1.602029;
  smac_path.poses[22].pose.orientation.x = 0.000000;
  smac_path.poses[22].pose.orientation.y = 0.000000;
  smac_path.poses[22].pose.orientation.z = 0.608761;
  smac_path.poses[22].pose.orientation.w = 0.793353;
  smac_path.poses[23].pose.position.x = 2.062939;
  smac_path.poses[23].pose.position.y = 1.702892;
  smac_path.poses[23].pose.orientation.x = 0.000000;
  smac_path.poses[23].pose.orientation.y = 0.000000;
  smac_path.poses[23].pose.orientation.z = 0.608761;
  smac_path.poses[23].pose.orientation.w = 0.793353;
  smac_path.poses[24].pose.position.x = 2.088483;
  smac_path.poses[24].pose.position.y = 1.772004;
  smac_path.poses[24].pose.orientation.x = 0.000000;
  smac_path.poses[24].pose.orientation.y = 0.000000;
  smac_path.poses[24].pose.orientation.z = 0.500000;
  smac_path.poses[24].pose.orientation.w = 0.866025;
  smac_path.poses[25].pose.position.x = 2.123730;
  smac_path.poses[25].pose.position.y = 1.836797;
  smac_path.poses[25].pose.orientation.x = 0.000000;
  smac_path.poses[25].pose.orientation.y = 0.000000;
  smac_path.poses[25].pose.orientation.z = 0.500000;
  smac_path.poses[25].pose.orientation.w = 0.866025;
  smac_path.poses[26].pose.position.x = 2.150105;
  smac_path.poses[26].pose.position.y = 1.905596;
  smac_path.poses[26].pose.orientation.x = 0.000000;
  smac_path.poses[26].pose.orientation.y = 0.000000;
  smac_path.poses[26].pose.orientation.z = 0.573576;
  smac_path.poses[26].pose.orientation.w = 0.819152;
  smac_path.poses[27].pose.position.x = 2.163413;
  smac_path.poses[27].pose.position.y = 1.978065;
  smac_path.poses[27].pose.orientation.x = 0.000000;
  smac_path.poses[27].pose.orientation.y = 0.000000;
  smac_path.poses[27].pose.orientation.z = 0.642788;
  smac_path.poses[27].pose.orientation.w = 0.766044;
  smac_path.poses[28].pose.position.x = 2.163204;
  smac_path.poses[28].pose.position.y = 2.051746;
  smac_path.poses[28].pose.orientation.x = 0.000000;
  smac_path.poses[28].pose.orientation.y = 0.000000;
  smac_path.poses[28].pose.orientation.z = 0.737277;
  smac_path.poses[28].pose.orientation.w = 0.675590;
  smac_path.poses[29].pose.position.x = 2.149483;
  smac_path.poses[29].pose.position.y = 2.124139;
  smac_path.poses[29].pose.orientation.x = 0.000000;
  smac_path.poses[29].pose.orientation.y = 0.000000;
  smac_path.poses[29].pose.orientation.z = 0.793353;
  smac_path.poses[29].pose.orientation.w = 0.608761;
  smac_path.poses[30].pose.position.x = 2.122717;
  smac_path.poses[30].pose.position.y = 2.192787;
  smac_path.poses[30].pose.orientation.x = 0.000000;
  smac_path.poses[30].pose.orientation.y = 0.000000;
  smac_path.poses[30].pose.orientation.z = 0.843391;
  smac_path.poses[30].pose.orientation.w = 0.537300;
  smac_path.poses[31].pose.position.x = 2.083813;
  smac_path.poses[31].pose.position.y = 2.255361;
  smac_path.poses[31].pose.orientation.x = 0.000000;
  smac_path.poses[31].pose.orientation.y = 0.000000;
  smac_path.poses[31].pose.orientation.z = 0.887011;
  smac_path.poses[31].pose.orientation.w = 0.461749;
  smac_path.poses[32].pose.position.x = 2.034093;
  smac_path.poses[32].pose.position.y = 2.309737;
  smac_path.poses[32].pose.orientation.x = 0.000000;
  smac_path.poses[32].pose.orientation.y = 0.000000;
  smac_path.poses[32].pose.orientation.z = 0.923880;
  smac_path.poses[32].pose.orientation.w = 0.382683;
  smac_path.poses[33].pose.position.x = 2.039769;
  smac_path.poses[33].pose.position.y = 2.309702;
  smac_path.poses[33].pose.orientation.x = 0.000000;
  smac_path.poses[33].pose.orientation.y = 0.000000;
  smac_path.poses[33].pose.orientation.z = 0.953717;
  smac_path.poses[33].pose.orientation.w = 0.300706;
  smac_path.poses[34].pose.position.x = 2.105753;
  smac_path.poses[34].pose.position.y = 2.276914;
  smac_path.poses[34].pose.orientation.x = 0.000000;
  smac_path.poses[34].pose.orientation.y = 0.000000;
  smac_path.poses[34].pose.orientation.z = 0.976296;
  smac_path.poses[34].pose.orientation.w = 0.216440;
  smac_path.poses[35].pose.position.x = 2.176632;
  smac_path.poses[35].pose.position.y = 2.256786;
  smac_path.poses[35].pose.orientation.x = 0.000000;
  smac_path.poses[35].pose.orientation.y = 0.000000;
  smac_path.poses[35].pose.orientation.z = 0.991445;
  smac_path.poses[35].pose.orientation.w = 0.130526;
  smac_path.poses[36].pose.position.x = 2.250000;
  smac_path.poses[36].pose.position.y = 2.250000;
  smac_path.poses[36].pose.orientation.x = 0.000000;
  smac_path.poses[36].pose.orientation.y = -0.000000;
  smac_path.poses[36].pose.orientation.z = 1.000000;
  smac_path.poses[36].pose.orientation.w = -0.000000;

  // Check that we accurately detect that this path has a reversing segment
  auto path_segs = smoother->findDirectionalPathSegmentsWrapper(smac_path);
  EXPECT_TRUE(path_segs.size() == 2u || path_segs.size() == 3u);

  // Test smoother, should succeed with same number of points
  // and shorter overall length, while still being collision free.
  double initial_length = smoother->getPathLength(smac_path);
  auto path_size_in = smac_path.poses.size();
  EXPECT_TRUE(smoother->smooth(smac_path, max_time));
  EXPECT_EQ(smac_path.poses.size(), path_size_in);  // Should have same number of poses
  EXPECT_LT(smoother->getPathLength(smac_path), initial_length);  // Should be shorter
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
