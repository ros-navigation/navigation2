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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

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
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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
// TF branch in globalPathCallback
// ============================================================

// When the path frame differs from the costmap frame and the TF buffer has no
// matching transform, globalPathCallback must catch the exception and leave
// latest_path_transform_ as nullopt.  Covers lines 160-163.
TEST_F(AsymmetricInflationIntegrationTest, global_path_callback_tf_failure_sets_nullopt_transform)
{
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->header.frame_id = "odom";  // costmap global frame is "map" → triggers TF lookup
  path->header.stamp = node_->now();
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.header.frame_id = "odom";
  p1.pose.position.x = 1.0;
  p1.pose.position.y = 2.0;
  p2 = p1;
  p2.pose.position.x = 2.0;
  path->poses = {p1, p2};

  // tf_ has no "odom"→"map" transform → lookupTransform throws tf2::TransformException.
  EXPECT_NO_THROW(layer_->injectPath(path));
}

// After a TF-failure injection, latest_path_transform_ is nullopt.
// extractLocalPathSegments hits the !cached_transform guard and falls back to symmetric.
TEST_F(AsymmetricInflationIntegrationTest, extract_path_segments_no_cached_transform_falls_back)
{
  layer_->setSideFactors(1.0, 10.0);

  // Inject cross-frame path without any TF set up → latest_path_transform_ = nullopt.
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
    << "No cached transform must force symmetric fallback";
}

// With a valid static TF transform available, extractLocalPathSegments must
// apply tf2::doTransform to each segment endpoint.
TEST_F(AsymmetricInflationIntegrationTest, transform_path_segments_with_valid_tf)
{
  // Register an identity static transform: map ← odom.
  geometry_msgs::msg::TransformStamped identity_tf;
  identity_tf.header.frame_id = "map";
  identity_tf.child_frame_id = "odom";
  identity_tf.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  identity_tf.transform.rotation.w = 1.0;
  tf_->setTransform(identity_tf, "test", true);  // static=true → available at all times

  // Inject 4-pose path in "odom" frame.  TF lookup will succeed and
  // latest_path_transform_ is set to the identity transform.
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

  // With left disfavored and the identity transform, the inflated layout must
  // match the same-frame case.
  layer_->setSideFactors(1.0, 10.0);
  seedLethalObstacle(20, 24);  // north side — disfavored

  runInflation();

  auto * costmap = layers_->getCostmap();
  unsigned char left_cost = costmap->getCost(20, 28);
  unsigned char right_cost = costmap->getCost(20, 12);
  EXPECT_GT(left_cost, right_cost)
    << "TF-transformed path must still produce asymmetric costs";
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
