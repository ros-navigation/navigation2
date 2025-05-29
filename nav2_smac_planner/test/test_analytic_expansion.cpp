// Copyright (c) 2024 Your Name
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

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/analytic_expansion.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"

class AnalyticExpansionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    collision_checker_ = std::make_shared<nav2_smac_planner::GridCollisionChecker>(
      std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0.0, 0.0));
    
    // Create an analytic expansion instance for testing
    expander_ = std::make_unique<nav2_smac_planner::AnalyticExpansion<nav2_smac_planner::NodeHybrid>>(
      collision_checker_, 72.0, false, 10);
  }

  std::shared_ptr<nav2_smac_planner::GridCollisionChecker> collision_checker_;
  std::unique_ptr<nav2_smac_planner::AnalyticExpansion<nav2_smac_planner::NodeHybrid>> expander_;
};

TEST_F(AnalyticExpansionTest, TestAnalyticExpansionNodesStructure)
{
  // Test the new AnalyticExpansionNodes structure
  nav2_smac_planner::AnalyticExpansion<nav2_smac_planner::NodeHybrid>::AnalyticExpansionNodes nodes;
  
  // Verify initial values
  EXPECT_EQ(nodes.nodes.size(), 0u);
  EXPECT_EQ(nodes.direction_changes, 0);
  
  // Set direction changes
  nodes.setDirectionChanges(3);
  EXPECT_EQ(nodes.direction_changes, 3);
  
  // Add nodes and verify
  auto node_ptr = std::make_shared<nav2_smac_planner::NodeHybrid>(1);
  nav2_smac_planner::NodeHybrid::Coordinates initial_coords{1.0, 2.0, 0.0};
  nav2_smac_planner::NodeHybrid::Coordinates proposed_coords{3.0, 4.0, 0.5};
  
  nodes.add(node_ptr, initial_coords, proposed_coords);
  EXPECT_EQ(nodes.nodes.size(), 1u);
  EXPECT_EQ(nodes.nodes[0].node, node_ptr);
  EXPECT_FLOAT_EQ(nodes.nodes[0].initial_coords.x, 1.0);
  EXPECT_FLOAT_EQ(nodes.nodes[0].initial_coords.y, 2.0);
  EXPECT_FLOAT_EQ(nodes.nodes[0].proposed_coords.x, 3.0);
  EXPECT_FLOAT_EQ(nodes.nodes[0].proposed_coords.y, 4.0);
}

// Test the countDirectionChanges method by directly calling it
TEST_F(AnalyticExpansionTest, TestCountDirectionChanges)
{
  // Create a mock Reeds-Shepp path with different patterns
  // For RS paths, the direction is indicated by the sign of the length
  
  // Test case 1: No direction changes (all forward)
  {
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath path;
    path.length_[0] = 1.0;  // Forward
    path.length_[1] = 2.0;  // Forward
    path.length_[2] = 0.0;  // No segment
    path.length_[3] = 0.0;  // No segment
    path.length_[4] = 0.0;  // No segment
    
    int changes = expander_->countDirectionChanges(path);
    EXPECT_EQ(changes, 0);
  }
  
  // Test case 2: One direction change
  {
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath path;
    path.length_[0] = 1.0;   // Forward
    path.length_[1] = -2.0;  // Backward
    path.length_[2] = 0.0;   // No segment
    path.length_[3] = 0.0;   // No segment
    path.length_[4] = 0.0;   // No segment
    
    int changes = expander_->countDirectionChanges(path);
    EXPECT_EQ(changes, 1);
  }
  
  // Test case 3: Multiple direction changes
  {
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath path;
    path.length_[0] = 1.0;   // Forward
    path.length_[1] = -2.0;  // Backward
    path.length_[2] = 3.0;   // Forward
    path.length_[3] = -1.0;  // Backward
    path.length_[4] = 0.0;   // No segment
    
    int changes = expander_->countDirectionChanges(path);
    EXPECT_EQ(changes, 3);
  }
  
  // Test case 4: Empty path (all zeros)
  {
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath path;
    path.length_[0] = 0.0;
    path.length_[1] = 0.0;
    path.length_[2] = 0.0;
    path.length_[3] = 0.0;
    path.length_[4] = 0.0;
    
    int changes = expander_->countDirectionChanges(path);
    EXPECT_EQ(changes, 0);
  }
  
  // Test case 5: Only backward segments (no changes)
  {
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath path;
    path.length_[0] = -1.0;
    path.length_[1] = -2.0;
    path.length_[2] = 0.0;
    path.length_[3] = 0.0;
    path.length_[4] = 0.0;
    
    int changes = expander_->countDirectionChanges(path);
    EXPECT_EQ(changes, 0);
  }
}

// This test would require mocking a lot of things to test the path refinement
// behavior. In a real setting, this would be an integration test combining
// the analytic expansion with the path refinement process.
TEST_F(AnalyticExpansionTest, TestRefineAnalyticPathWithDirectionChanges)
{
  // This is a placeholder. In a real scenario, this would:
  // 1. Create a scenario where multiple analytic paths exist
  // 2. Some paths would have fewer direction changes but higher cost
  // 3. Verify the algorithm chooses the path with the right balance
  // 4. Confirm that the path with excessive direction changes is rejected
  
  // For now, we'll just verify our testing framework is set up correctly
  EXPECT_TRUE(expander_ != nullptr);
  EXPECT_TRUE(collision_checker_ != nullptr);
}
