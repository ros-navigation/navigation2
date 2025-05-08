// Copyright (c) 2025, Open Navigation LLC
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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_route/goal_intent_extractor.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

TEST(GoalIntentSearchTest, test_obj_lifecycle)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap = nullptr;
  GoalIntentSearch::BreadthFirstSearch bfs(costmap);
  EXPECT_EQ(bfs.getClosestNodeIdx(), 0u);
  GoalIntentSearch::LoSCollisionChecker los_checker(costmap);
}

TEST(GoalIntentSearchTest, test_los_checker)
{
  // Create a demo costmap: * = 100, - = 0, / = 254
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  auto costmap =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0.0, 0.0, 0);
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap->setCost(i, j, 254);
    }
  }
  for (unsigned int i = 0; i < 40; ++i) {
    for (unsigned int j = 0; j < 100; ++j) {
      costmap->setCost(i, j, 100);
    }
  }

  GoalIntentSearch::LoSCollisionChecker los_checker(costmap);

  // Valid request
  geometry_msgs::msg::Point start, end;
  start.x = 1.0;
  start.y = 1.0;
  end.x = 1.0;
  end.y = 9.0;
  EXPECT_TRUE(los_checker.worldToMap(start, end));
  EXPECT_FALSE(los_checker.isInCollision());

  // In collision request
  start.x = 1.0;
  start.y = 1.0;
  end.x = 9.0;
  end.y = 9.0;
  EXPECT_TRUE(los_checker.worldToMap(start, end));
  EXPECT_TRUE(los_checker.isInCollision());

  // Off of costmap request
  end.x = 11.0;
  end.y = 11.0;
  EXPECT_FALSE(los_checker.worldToMap(start, end));
}

TEST(GoalIntentSearchTest, test_breadth_first_search)
{
  // Create a demo costmap: * = 100, - = 0, / = 254
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  auto costmap =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0.0, 0.0, 0);
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap->setCost(i, j, 254);
    }
  }
  for (unsigned int i = 0; i < 40; ++i) {
    for (unsigned int j = 0; j < 100; ++j) {
      costmap->setCost(i, j, 100);
    }
  }

  GoalIntentSearch::BreadthFirstSearch bfs(costmap);

  std::vector<geometry_msgs::msg::PoseStamped> candidate_nodes;
  geometry_msgs::msg::PoseStamped target_node;
  target_node.header.frame_id = "map";
  target_node.pose.position.x = 1.0;
  target_node.pose.position.y = 1.0;

  geometry_msgs::msg::PoseStamped node;
  node.header.frame_id = "map";
  node.pose.position.x = 9.0;
  node.pose.position.y = 9.0;
  candidate_nodes.push_back(node);  // First is far

  node.pose.position.x = 5.0;
  node.pose.position.y = 5.0;
  candidate_nodes.push_back(node);  // Second is close but in collision

  node.pose.position.x = 1.0;
  node.pose.position.y = 9.0;
  candidate_nodes.push_back(node);  // Third is closest valid

  // Test successful search
  EXPECT_TRUE(bfs.search(target_node, candidate_nodes));
  EXPECT_EQ(bfs.getClosestNodeIdx(), 2u);  // Should find the third

  // Test max iterations
  int max_it = 1;
  EXPECT_FALSE(bfs.search(target_node, candidate_nodes, max_it));
  EXPECT_EQ(bfs.getClosestNodeIdx(), 0u);

  // Test reference off of the map
  target_node.pose.position.x = 10000.0;
  EXPECT_FALSE(bfs.search(target_node, candidate_nodes));

  // Test candidates off of the map
  target_node.pose.position.x = 1.0;
  for (auto & node_c : candidate_nodes) {
    node_c.pose.position.x = 10000.0;
  }
  EXPECT_FALSE(bfs.search(target_node, candidate_nodes));

  // Test empty candidates
  candidate_nodes.clear();
  EXPECT_FALSE(bfs.search(target_node, candidate_nodes));
}
