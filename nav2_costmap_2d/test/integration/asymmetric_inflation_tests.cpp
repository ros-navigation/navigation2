// Copyright (c) 2026 Marc Blöchlinger
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

// Integration tests for AsymmetricInflationLayer.
//
// These tests stand up a minimal LayeredCostmap with the asymmetric layer
// attached and exercise updateCosts() on synthetic obstacle layouts.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using geometry_msgs::msg::Point;

class TestableAsymmetricInflationLayer : public nav2_costmap_2d::AsymmetricInflationLayer
{
public:
  void injectPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    globalPathCallback(msg);
  }

  void setSideFactors(double left, double right)
  {
    cost_scaling_factor_left_ = left;
    cost_scaling_factor_right_ = right;
    cost_scaling_factor_ = std::max(left, right);
    need_reinflation_ = true;
    setCurrent(false);
    matchSize();
  }

  void setGoalDistanceThreshold(double t) {goal_distance_threshold_ = t;}
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcpp_fixture;

class AsymmetricInflationIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(
    {
      {"asymmetric_inflation_layer.enabled", true},
      {"asymmetric_inflation_layer.inflation_radius", 0.55},
      {"asymmetric_inflation_layer.cost_scaling_factor_left", 10.0},
      {"asymmetric_inflation_layer.cost_scaling_factor_right", 10.0},
      {"asymmetric_inflation_layer.inflate_unknown", false},
      {"asymmetric_inflation_layer.inflate_around_unknown", false},
      {"asymmetric_inflation_layer.plan_topic", std::string("plan")},
      {"asymmetric_inflation_layer.goal_distance_threshold", 1.5},
      {"map_topic", std::string("map")},
      {"track_unknown_space", false},
      {"use_maximum", false},
      {"lethal_cost_threshold", 100},
      {"unknown_cost_value", static_cast<unsigned char>(0xff)},
      {"trinary_costmap", true},
      {"transform_tolerance", 0.3},
      {"observation_sources", std::string("")},
    });

    node_ = std::make_shared<nav2::LifecycleNode>(
      "asymmetric_inflation_test_node", "", options);

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    // 40x40 cells, 0.1 m resolution, origin at (0,0) — world range [0, 4] m.
    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layers_->resizeMap(40, 40, 0.1, 0.0, 0.0);

    // Square footprint with corners at ±0.1 m → inscribed_radius ≈ 0.1 m.
    std::vector<Point> footprint;
    Point p;
    p.x = 0.1; p.y = 0.1; footprint.push_back(p);
    p.x = 0.1; p.y = -0.1; footprint.push_back(p);
    p.x = -0.1; p.y = -0.1; footprint.push_back(p);
    p.x = -0.1; p.y = 0.1; footprint.push_back(p);
    layers_->setFootprint(footprint);

    layer_ = std::make_shared<TestableAsymmetricInflationLayer>();
    layer_->initialize(
      layers_.get(), "asymmetric_inflation_layer", tf_.get(), node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
  }

  void seedLethalObstacle(unsigned int mx, unsigned int my)
  {
    layers_->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  void runInflation()
  {
    auto * costmap = layers_->getCostmap();
    unsigned int w = costmap->getSizeInCellsX();
    unsigned int h = costmap->getSizeInCellsY();
    layer_->updateCosts(*costmap, 0, 0, w, h);
  }

  // Path along y=2.0 with 4 poses at x = {0.5, 1.5, 2.5, 3.5}.
  // Robot is at (0,0) by default; goal is at (3.5, 2.0), dist ≈ 4.03 m.
  void injectStraightPath()
  {
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = "map";

    for (double x : {0.5, 1.5, 2.5, 3.5}) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = 2.0;
      path->poses.push_back(pose);
    }

    layer_->injectPath(path);
  }

  // Path with only 1 pose — triggers the "< 2 poses" early-return in
  // extractLocalPathSegments, making the layer behave symmetrically.
  void injectShortPath()
  {
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 2.0;
    pose.pose.position.y = 2.0;
    path->poses.push_back(pose);
    layer_->injectPath(path);
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<TestableAsymmetricInflationLayer> layer_;
};

// ============================================================
// Existing tests (symmetric fallbacks and basic asymmetry)
// ============================================================

// Without a published path the layer must produce a symmetric symmetric baseline.
TEST_F(AsymmetricInflationIntegrationTest, no_path_produces_symmetric_output)
{
  seedLethalObstacle(20, 20);
  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left = costmap->getCost(15, 20);
  unsigned char right = costmap->getCost(25, 20);
  EXPECT_EQ(left, right) << "Without a path, costs must be symmetric";
  EXPECT_GT(left, nav2_costmap_2d::FREE_SPACE)
    << "Standalone asymmetric layer must still write the symmetric baseline";
}

// Equal per-side factors short-circuit the asymmetric pass even when a path
// is present; the output must stay symmetric.
TEST_F(AsymmetricInflationIntegrationTest, equal_sides_produce_symmetric_output)
{
  injectStraightPath();
  seedLethalObstacle(20, 20);
  seedLethalObstacle(20, 30);

  runInflation();

  auto * costmap = layers_->getCostmap();
  for (int dy = -5; dy <= 5; ++dy) {
    unsigned char left = costmap->getCost(15, 20 + dy);
    unsigned char right = costmap->getCost(25, 20 + dy);
    EXPECT_EQ(left, right) << "Asymmetry with equal sides (y_off=" << dy << ")";
  }
}

// With unequal factors the disfavored side must receive higher costs than the
// favored side at symmetric distances from the obstacle.
TEST_F(AsymmetricInflationIntegrationTest, unequal_sides_raise_costs_from_standalone_layer)
{
  layer_->setSideFactors(1.0, 10.0);
  injectStraightPath();

  seedLethalObstacle(20, 24);  // left/north of the path, within inflation radius
  seedLethalObstacle(20, 16);  // right/south of the path, within inflation radius

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left_side_cost = costmap->getCost(20, 28);
  unsigned char right_side_cost = costmap->getCost(20, 12);
  EXPECT_GT(left_side_cost, right_side_cost)
    << "Smaller left-side decay factor must raise costs above the symmetric baseline";
}

TEST_F(AsymmetricInflationIntegrationTest, implements_inflation_layer_interface)
{
  std::shared_ptr<nav2_costmap_2d::Layer> base_layer = layer_;
  auto inflation_layer =
    std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayerInterface>(base_layer);

  ASSERT_NE(inflation_layer, nullptr);
  EXPECT_DOUBLE_EQ(inflation_layer->getInflationRadius(), 0.55);
  EXPECT_DOUBLE_EQ(inflation_layer->getCostScalingFactor(), 10.0);
}

// A path with fewer than 2 poses causes extractLocalPathSegments to return
// an empty vector → only the symmetric Pass 1 runs.
TEST_F(AsymmetricInflationIntegrationTest, short_path_falls_back_to_symmetric)
{
  layer_->setSideFactors(1.0, 10.0);
  injectShortPath();
  seedLethalObstacle(20, 20);

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left = costmap->getCost(15, 20);
  unsigned char right = costmap->getCost(25, 20);
  EXPECT_EQ(left, right)
    << "A 1-pose path must fall back to symmetric inflation";
  EXPECT_GT(left, nav2_costmap_2d::FREE_SPACE)
    << "Symmetric baseline must still write non-zero costs";
}

// When the robot is within goal_distance_threshold of the path's goal,
// extractLocalPathSegments returns empty → symmetric output despite unequal factors.
TEST_F(AsymmetricInflationIntegrationTest, near_goal_disables_asymmetry)
{
  layer_->setSideFactors(1.0, 10.0);
  // Raise threshold to 5.0 m; the default robot position is (0,0) and the
  // goal of injectStraightPath is (3.5, 2.0) — dist ≈ 4.03 m < 5.0 m.
  layer_->setGoalDistanceThreshold(5.0);
  injectStraightPath();

  seedLethalObstacle(20, 24);  // left side
  seedLethalObstacle(20, 16);  // right side

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left_cost = costmap->getCost(20, 28);
  unsigned char right_cost = costmap->getCost(20, 12);
  EXPECT_EQ(left_cost, right_cost)
    << "Near-goal threshold must disable asymmetry → symmetric costs";
}

// Swapping the disfavored side flips which side receives higher costs.
TEST_F(AsymmetricInflationIntegrationTest, right_side_disfavored_raises_right_costs)
{
  layer_->setSideFactors(10.0, 1.0);  // right disfavored (c_right < c_left)
  injectStraightPath();

  seedLethalObstacle(20, 16);  // right/south side obstacle
  seedLethalObstacle(20, 24);  // left/north side obstacle (favored — low asymmetry)

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char right_side_cost = costmap->getCost(20, 12);
  unsigned char left_side_cost = costmap->getCost(20, 28);
  EXPECT_GT(right_side_cost, left_side_cost)
    << "Smaller right-side decay factor must raise costs on the right side";
}

// An obstacle at the map edge (i == 0) must be detected as a boundary cell via
// the is_on_map_edge check and inflated by Pass 2.
TEST_F(AsymmetricInflationIntegrationTest, map_edge_obstacle_is_inflated)
{
  layer_->setSideFactors(1.0, 10.0);  // left disfavored
  injectStraightPath();

  // Cell (0, 20) is at world (0.05, 2.05) — on the left (north) side of the
  // y=2.0 path and on the x=0 map edge.
  seedLethalObstacle(0, 20);

  runInflation();

  auto * costmap = layers_->getCostmap();
  // With c_side=1.0 and inscribed_radius=0, the cost 4 cells east of the edge
  // obstacle is: 252 * exp(−1.0 * 0.4) ≈ 169.  A symmetric-only run would give ≈1.
  // Any value substantially above the symmetric baseline confirms Pass 2 seeded
  // the edge obstacle via the is_on_map_edge branch.
  EXPECT_GT(static_cast<int>(costmap->getCost(4, 20)), 100)
    << "Edge obstacle must be seeded by Pass 2 via the is_on_map_edge boundary check";
}

// With known parameters the inflated cost at an exact distance must match the
// formula:  cost = floor(252 * exp(−c_side * (d_cells * res − insc_r))).
//
// Setup: c_left=3.0, c_right=10.0 → c_side=3.0, inflation_radius=0.55 m,
//        inscribed_radius=0.0 m (setFootprint is called before the layer is
//        added to plugins, so onFootprintChanged never fires → inscribed_radius_
//        stays at its constructor default of 0.0), resolution=0.1 m,
//        cell_inflation_radius=ceil(0.55/0.1)=6.
// Obstacle at (20,24), target at (20,29): distance = 5 cells = 0.5 m.
//   expected = floor(252 * exp(−3.0 * (0.5 − 0.0)))
//            = floor(252 * exp(−1.5)) ≈ floor(56.2) = 56.
TEST_F(AsymmetricInflationIntegrationTest, cost_at_known_distance_matches_formula)
{
  layer_->setSideFactors(3.0, 10.0);
  injectStraightPath();
  seedLethalObstacle(20, 24);  // left side, 0.45 m north of path — within 0.55 m radius

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char actual = costmap->getCost(20, 29);
  // Allow ±2 for fixed-point quantisation inside the LUT.
  EXPECT_NEAR(static_cast<int>(actual), 56, 2)
    << "Cost at 5 cells from disfavored-side obstacle must match the decay formula";
}

// applyInflation uses max(old, new) semantics: a cell whose existing cost
// already exceeds what Pass 2 would write must not be lowered.
TEST_F(AsymmetricInflationIntegrationTest, max_cost_not_overwritten_by_inflation)
{
  layer_->setSideFactors(1.0, 10.0);
  injectStraightPath();
  seedLethalObstacle(20, 24);  // left side obstacle

  // Pre-set cell (20,28) to 200. Pass 2 would write ≈186 there
  // (252 * exp(−1.0 * 0.3) ≈ 186 < 200), so max semantics must preserve 200.
  layers_->getCostmap()->setCost(20, 28, 200);

  runInflation();

  EXPECT_EQ(static_cast<int>(layers_->getCostmap()->getCost(20, 28)), 200)
    << "applyInflation must not overwrite a cell whose existing cost exceeds the new cost";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
