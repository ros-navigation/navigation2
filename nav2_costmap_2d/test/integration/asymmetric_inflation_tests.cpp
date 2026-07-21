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
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using geometry_msgs::msg::Point;

class TestableAsymmetricInflationLayer : public nav2_costmap_2d::AsymmetricInflationLayer
{
public:
  void injectPath(const nav_msgs::msg::Path::ConstSharedPtr msg)
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

  // Expire node_ weak_ptr so that node_.lock() returns null — used by node-expired tests.
  void expireNode() {node_.reset();}

  double getCurrentRobotX() const {return current_robot_x_;}
  double getCurrentRobotY() const {return current_robot_y_;}

  void callOnFootprintChanged() {onFootprintChanged();}

  rcl_interfaces::msg::SetParametersResult callValidate(
    const std::vector<rclcpp::Parameter> & p)
  {
    return validateParameterUpdatesCallback(p);
  }

  void callUpdate(const std::vector<rclcpp::Parameter> & p)
  {
    updateParametersCallback(p);
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcpp_fixture;

// ============================================================
// Helper: build a default valid parameter set for initialize()
// ============================================================
static rclcpp::NodeOptions makeNodeOptions(
  const std::vector<rclcpp::Parameter> & overrides = {})
{
  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("asymmetric_inflation_layer.enabled", true),
    rclcpp::Parameter("asymmetric_inflation_layer.inflation_radius", 0.55),
    rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor_left", 10.0),
    rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor_right", 10.0),
    rclcpp::Parameter("asymmetric_inflation_layer.inflate_unknown", false),
    rclcpp::Parameter("asymmetric_inflation_layer.inflate_around_unknown", false),
    rclcpp::Parameter("asymmetric_inflation_layer.plan_topic", std::string("plan")),
    rclcpp::Parameter("asymmetric_inflation_layer.goal_distance_threshold", 1.5),
    rclcpp::Parameter("asymmetric_inflation_layer.num_threads", -1),
    rclcpp::Parameter("map_topic", std::string("map")),
    rclcpp::Parameter("track_unknown_space", false),
    rclcpp::Parameter("use_maximum", false),
    rclcpp::Parameter("lethal_cost_threshold", 100),
    rclcpp::Parameter("unknown_cost_value", static_cast<unsigned char>(0xff)),
    rclcpp::Parameter("trinary_costmap", true),
    rclcpp::Parameter("transform_tolerance", 0.3),
    rclcpp::Parameter("observation_sources", std::string("")),
  };
  // Apply overrides (later entries in the vector take precedence via parameter_overrides)
  for (const auto & ov : overrides) {
    params.push_back(ov);
  }
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides(params);
  return options;
}

// ============================================================
// AsymmetricInflationLayerInitTest — standalone tests
// ============================================================

// onInitialize() must throw if node_ is expired.
TEST(AsymmetricInflationLayerInitTest, onInitialize_throws_if_node_expired)
{
  TestableAsymmetricInflationLayer layer;
  layer.expireNode();
  EXPECT_THROW(layer.onInitialize(), std::runtime_error);
}

// activate() must throw if node_ is expired.
TEST(AsymmetricInflationLayerInitTest, activate_throws_if_node_expired)
{
  TestableAsymmetricInflationLayer layer;
  layer.expireNode();
  EXPECT_THROW(layer.activate(), std::runtime_error);
}

// onInitialize() must throw for inflation_radius < 0
TEST(AsymmetricInflationLayerInitTest, init_throws_on_negative_inflation_radius)
{
  auto options = makeNodeOptions(
    {rclcpp::Parameter("asymmetric_inflation_layer.inflation_radius", -0.1)});
  auto node = std::make_shared<nav2::LifecycleNode>("init_test_infl", "", options);
  auto tf = nav2::create_transform_buffer(node);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 0.1, 0.0, 0.0);
  auto layer = std::make_shared<TestableAsymmetricInflationLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "asymmetric_inflation_layer", tf.get(), node, nullptr),
    std::runtime_error);
}

// onInitialize() must throw for cost_scaling_factor_left < 0
TEST(AsymmetricInflationLayerInitTest, init_throws_on_negative_left_scaling)
{
  auto options = makeNodeOptions(
    {rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor_left", -1.0)});
  auto node = std::make_shared<nav2::LifecycleNode>("init_test_left", "", options);
  auto tf = nav2::create_transform_buffer(node);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 0.1, 0.0, 0.0);
  auto layer = std::make_shared<TestableAsymmetricInflationLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "asymmetric_inflation_layer", tf.get(), node, nullptr),
    std::runtime_error);
}

// onInitialize() must throw for cost_scaling_factor_right < 0
TEST(AsymmetricInflationLayerInitTest, init_throws_on_negative_right_scaling)
{
  auto options = makeNodeOptions(
    {rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor_right", -1.0)});
  auto node = std::make_shared<nav2::LifecycleNode>("init_test_right", "", options);
  auto tf = nav2::create_transform_buffer(node);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 0.1, 0.0, 0.0);
  auto layer = std::make_shared<TestableAsymmetricInflationLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "asymmetric_inflation_layer", tf.get(), node, nullptr),
    std::runtime_error);
}

// onInitialize() must throw for goal_distance_threshold < 0
TEST(AsymmetricInflationLayerInitTest, init_throws_on_negative_goal_distance_threshold)
{
  auto options = makeNodeOptions(
    {rclcpp::Parameter("asymmetric_inflation_layer.goal_distance_threshold", -0.1)});
  auto node = std::make_shared<nav2::LifecycleNode>("init_test_goal", "", options);
  auto tf = nav2::create_transform_buffer(node);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 0.1, 0.0, 0.0);
  auto layer = std::make_shared<TestableAsymmetricInflationLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "asymmetric_inflation_layer", tf.get(), node, nullptr),
    std::runtime_error);
}

// onInitialize() must throw for num_threads < -1
TEST(AsymmetricInflationLayerInitTest, init_throws_on_invalid_num_threads)
{
  auto options = makeNodeOptions(
    {rclcpp::Parameter("asymmetric_inflation_layer.num_threads", -2)});
  auto node = std::make_shared<nav2::LifecycleNode>("init_test_threads", "", options);
  auto tf = nav2::create_transform_buffer(node);
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  layers.resizeMap(10, 10, 0.1, 0.0, 0.0);
  auto layer = std::make_shared<TestableAsymmetricInflationLayer>();
  EXPECT_THROW(
    layer->initialize(&layers, "asymmetric_inflation_layer", tf.get(), node, nullptr),
    std::runtime_error);
}

// ============================================================
// AsymmetricInflationIntegrationTest — full LayeredCostmap setup
// ============================================================

class AsymmetricInflationIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>(
      "asymmetric_inflation_test_node", "", makeNodeOptions());

    tf_ = nav2::create_transform_buffer(node_);

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
  nav2::TransformBuffer::SharedPtr tf_;
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

// ============================================================
// lifecycle API
// ============================================================

// activate() registers parameter callbacks; deactivate() removes them.
TEST_F(AsymmetricInflationIntegrationTest, activate_and_deactivate_succeed)
{
  EXPECT_NO_THROW(layer_->activate());
  EXPECT_NO_THROW(layer_->deactivate());
}

// updateBounds() stores the robot pose for the goal-proximity check.
TEST_F(AsymmetricInflationIntegrationTest, update_bounds_records_robot_pose)
{
  double min_x = 0.0, min_y = 0.0, max_x = 4.0, max_y = 4.0;
  layer_->updateBounds(1.5, 2.5, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(layer_->getCurrentRobotX(), 1.5);
  EXPECT_DOUBLE_EQ(layer_->getCurrentRobotY(), 2.5);
}

// onFootprintChanged() rebuilds the asymmetric cost LUT.
TEST_F(AsymmetricInflationIntegrationTest, footprint_change_recomputes_caches)
{
  EXPECT_NO_THROW(layer_->callOnFootprintChanged());
  seedLethalObstacle(20, 20);
  EXPECT_NO_THROW(runInflation());
  EXPECT_GT(
    static_cast<int>(layers_->getCostmap()->getCost(15, 20)),
    nav2_costmap_2d::FREE_SPACE)
    << "Inflation must still work after a footprint-change cache rebuild";
}

// ============================================================
// TF lookup in extractLocalPathSegments (per-cycle, live)
// ============================================================

// When the path frame differs from the costmap frame and the TF buffer has no matching
// transform, extractLocalPathSegments must catch the exception every cycle.
TEST_F(AsymmetricInflationIntegrationTest, extract_path_segments_tf_failure_falls_back_to_symmetric)
{
  layer_->setSideFactors(1.0, 10.0);

  // Inject cross-frame path without any TF set up → the per-cycle lookup in
  // extractLocalPathSegments() throws and the layer falls back to symmetric.
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "odom";
  path->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 1.0;
  p1.pose.position.y = 2.0;
  p2 = p1;
  p2.pose.position.x = 3.0;
  path->poses = {p1, p2};
  layer_->injectPath(path);

  seedLethalObstacle(20, 24);  // north of path
  seedLethalObstacle(20, 16);  // south of path
  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left_cost = costmap->getCost(20, 28);
  unsigned char right_cost = costmap->getCost(20, 12);
  EXPECT_EQ(left_cost, right_cost)
    << "TF lookup failure must force symmetric fallback";
}

// With a valid static TF that extractLocalPathSegments must map the local window back into the
// path frame to cull, then tf2::doTransform the surviving segment endpoints into the costmap frame.
TEST_F(AsymmetricInflationIntegrationTest, transform_path_segments_with_rotated_tf)
{
  // map ← odom transform: yaw rotation about z plus a translation. A non-90°
  // yaw makes the inverse-mapped costmap window a rotated (non-axis-aligned)
  // box, exercising the conservative-superset path-frame cull.
  const double yaw = 0.6;  // ~34.4°
  const double tx = 0.3, ty = -0.2;
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);

  geometry_msgs::msg::TransformStamped rotated_tf;
  rotated_tf.header.frame_id = "map";
  rotated_tf.child_frame_id = "odom";
  rotated_tf.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rotated_tf.transform.translation.x = tx;
  rotated_tf.transform.translation.y = ty;
  rotated_tf.transform.rotation.z = std::sin(yaw / 2.0);
  rotated_tf.transform.rotation.w = std::cos(yaw / 2.0);
  tf_->setTransform(rotated_tf, "test", true);  // static=true → available at all times

  // Inject 4-pose path in "odom" frame.
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "odom";
  path->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  for (double x : {0.5, 1.5, 2.5, 3.5}) {
    const double dx = x - tx;
    const double dy = 2.0 - ty;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = cos_y * dx + sin_y * dy;
    pose.pose.position.y = -sin_y * dx + cos_y * dy;
    path->poses.push_back(pose);
  }
  layer_->injectPath(path);

  // With left disfavored, the obstacle north of the (map-frame) path is on the
  // disfavored side and must receive higher costs than the favored south side.
  layer_->setSideFactors(1.0, 10.0);
  seedLethalObstacle(20, 24);  // north side — disfavored

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left_cost = costmap->getCost(20, 28);
  unsigned char right_cost = costmap->getCost(20, 12);
  EXPECT_GT(left_cost, right_cost)
    << "A rotated + translated TF path must still produce asymmetric costs";
}

// ============================================================
// Live per-cycle transform tracks map->odom drift
// ============================================================

// The path-frame -> costmap-frame transform must be looked up fresh every
// cycle so the asymmetric overlay follows map->odom drift that accrues between cycles.
// Publish the path once in "odom", then move the map->odom transform *between* two update cycles
// and confirm the disfavored-side overlay follows the new transform.
TEST_F(AsymmetricInflationIntegrationTest, live_transform_tracks_map_odom_drift)
{
  layer_->setSideFactors(1.0, 10.0);  // left (north) disfavored
  layer_->setGoalDistanceThreshold(0.0);  // never disable asymmetry near goal

  // Path published once, in "odom" frame, along y = 2.0. Never re-published.
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "odom";
  path->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  for (double x : {0.5, 1.5, 2.5, 3.5}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = x;
    pose.pose.position.y = 2.0;
    path->poses.push_back(pose);
  }
  layer_->injectPath(path);

  auto * costmap = layers_->getCostmap();
  const unsigned int w = costmap->getSizeInCellsX();
  const unsigned int h = costmap->getSizeInCellsY();

  auto publishMapOdomTransform = [this](double ty, int stamp_sec) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.frame_id = "map";
      tf_msg.child_frame_id = "odom";
      tf_msg.header.stamp = rclcpp::Time(stamp_sec, 0, RCL_ROS_TIME);
      tf_msg.transform.translation.y = ty;
      tf_msg.transform.rotation.w = 1.0;
      tf_->setTransform(tf_msg, "test", false /* dynamic, not static */);
    };

  // --- Cycle 1: map == odom (no drift yet). The north obstacle sits 0.45 m
  // north of the map-frame path (y=2.0) → disfavored side. The south obstacle
  // sits 0.35 m south → favored side.
  publishMapOdomTransform(0.0, 1);
  seedLethalObstacle(20, 24);  // world (2.05, 2.45) — 0.45 m north of path
  seedLethalObstacle(20, 16);  // world (2.05, 1.65) — 0.35 m south of path
  runInflation();

  unsigned char north_cost_cycle1 = costmap->getCost(20, 28);  // 0.4 m from north obstacle
  unsigned char south_cost_cycle1 = costmap->getCost(20, 12);  // 0.4 m from south obstacle
  EXPECT_GT(north_cost_cycle1, south_cost_cycle1)
    << "North obstacle must be on the disfavored side while map==odom";

  // --- Cycle 2: map->odom drifts by +0.9 m in y with a newer TF stamp.
  // The path is NOT republished, only the live transform changes.
  // The map-frame path is now at y=2.9, which flips the north obstacle to the favored side
  // and pushes the south obstacle outside the path's inflation radius entirely (neutral).
  // Reset + re-seed so cycle 1's overlay can't linger via max(old,new) of the second pass.
  costmap->resetMap(0, 0, w, h);
  seedLethalObstacle(20, 24);
  seedLethalObstacle(20, 16);
  publishMapOdomTransform(0.9, 2);
  runInflation();

  unsigned char north_cost_cycle2 = costmap->getCost(20, 28);
  unsigned char south_cost_cycle2 = costmap->getCost(20, 12);
  EXPECT_EQ(north_cost_cycle2, south_cost_cycle2)
    << "Live transform must track drift: the north obstacle is no longer "
       "disfavored once the path has effectively moved north of it";
  EXPECT_LT(north_cost_cycle2, north_cost_cycle1)
    << "The (formerly disfavored) north cell's cost must drop once drift "
       "moves it off the disfavored side";
}

// The per-cycle lookup means asymmetry can recover as soon as a transform becomes available for
// the first time, no need to re-publish the path.
TEST_F(AsymmetricInflationIntegrationTest, live_lookup_recovers_once_transform_becomes_available)
{
  layer_->setSideFactors(1.0, 10.0);
  layer_->setGoalDistanceThreshold(0.0);

  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "odom";
  path->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  for (double x : {0.5, 1.5, 2.5, 3.5}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = x;
    pose.pose.position.y = 2.0;
    path->poses.push_back(pose);
  }
  layer_->injectPath(path);  // no TF published yet

  auto * costmap = layers_->getCostmap();
  const unsigned int w = costmap->getSizeInCellsX();
  const unsigned int h = costmap->getSizeInCellsY();
  seedLethalObstacle(20, 24);  // north of path
  seedLethalObstacle(20, 16);  // south of path
  runInflation();

  unsigned char north_before = costmap->getCost(20, 28);
  unsigned char south_before = costmap->getCost(20, 12);
  EXPECT_EQ(north_before, south_before)
    << "No transform available yet must force symmetric fallback";

  // Now publish an identity map->odom transform.
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";
  tf_msg.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  tf_msg.transform.rotation.w = 1.0;
  tf_->setTransform(tf_msg, "test", false /* dynamic, not static */);

  costmap->resetMap(0, 0, w, h);
  seedLethalObstacle(20, 24);
  seedLethalObstacle(20, 16);
  runInflation();

  unsigned char north_after = costmap->getCost(20, 28);
  unsigned char south_after = costmap->getCost(20, 12);
  EXPECT_GT(north_after, south_after)
    << "Per-cycle lookup must pick up the transform without re-publishing the path";
}

// ============================================================
// seedDistanceMap boundary checks
// ============================================================

// An interior obstacle (all 4 axis-aligned neighbours are lethal) must be
// skipped by seedDistanceMap.  Covers lines 512, 515.
TEST_F(AsymmetricInflationIntegrationTest, seed_distance_map_skips_interior_obstacles)
{
  layer_->setSideFactors(1.0, 10.0);
  injectStraightPath();

  // 3×3 lethal block; centre cell (20, 20) has all 4 neighbours lethal → interior.
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      seedLethalObstacle(20 + dx, 20 + dy);
    }
  }

  EXPECT_NO_THROW(runInflation());
}

// An obstacle whose world position puts it in a bucket not covered by any path
// segment must cause a spatial hash miss (it == spatial_hash.end()).
// Covers line 531.
TEST_F(AsymmetricInflationIntegrationTest, seed_distance_map_misses_obstacle_far_from_path)
{
  layer_->setSideFactors(1.0, 10.0);
  injectStraightPath();  // path at y=2.0; obstacle at (0,0) ≈ 1.95 m away

  seedLethalObstacle(0, 0);  // world (0.05, 0.05) — far outside inflation radius 0.55 m

  EXPECT_NO_THROW(runInflation());
  EXPECT_EQ(
    layers_->getCostmap()->getCost(0, 0),
    nav2_costmap_2d::LETHAL_OBSTACLE)
    << "Obstacle far from the path must remain lethal after inflation";
}

// ============================================================
// parameter callbacks
// ============================================================

// validateParameterUpdatesCallback delegates to the parent and returns success.
TEST_F(AsymmetricInflationIntegrationTest, validate_parameter_callback_delegates_to_parent)
{
  auto result = layer_->callValidate(
    {rclcpp::Parameter("asymmetric_inflation_layer.inflation_radius", 1.0)});
  EXPECT_TRUE(result.successful);
}

// updateParametersCallback: changing cost_scaling_factor_left triggers side_scaling_changed path.
TEST_F(AsymmetricInflationIntegrationTest, update_parameters_callback_left_factor_change)
{
  layer_->callUpdate(
    {rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor_left", 2.0)});
  injectStraightPath();
  seedLethalObstacle(20, 24);
  EXPECT_NO_THROW(runInflation());
}

// updateParametersCallback: changing cost_scaling_factor_right.
TEST_F(AsymmetricInflationIntegrationTest, update_parameters_callback_right_factor_change)
{
  layer_->callUpdate(
    {rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor_right", 2.0)});
  injectStraightPath();
  seedLethalObstacle(20, 16);
  EXPECT_NO_THROW(runInflation());
}

// updateParametersCallback: changing goal_distance_threshold
TEST_F(AsymmetricInflationIntegrationTest, update_parameters_callback_goal_threshold_change)
{
  layer_->callUpdate(
    {rclcpp::Parameter("asymmetric_inflation_layer.goal_distance_threshold", 3.0)});
  injectStraightPath();
  EXPECT_NO_THROW(runInflation());
}

// updateParametersCallback: the base cost_scaling_factor parameter (without
// _left/_right suffix) must be filtered out before forwarding to the parent.
TEST_F(AsymmetricInflationIntegrationTest, update_parameters_callback_filters_base_cost_scaling)
{
  layer_->callUpdate(
    {rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor", 5.0)});
  EXPECT_NO_THROW(runInflation());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
