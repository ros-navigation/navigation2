// Copyright (c) 2025 Berkan Tali
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

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/bounded_tracking_error_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2::LifecycleNode(name) {}

  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
  nav2::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {return nav2::CallbackReturn::SUCCESS;}
};

class TestableBoundedTrackingErrorLayer
  : public nav2_costmap_2d::BoundedTrackingErrorLayer
{
public:
  using WallPolygons = BoundedTrackingErrorLayer::WallPolygons;

  void testGetPathSegment(
    const nav_msgs::msg::Path & path, size_t idx,
    nav_msgs::msg::Path & segment)
  {getPathSegment(path, idx, segment);}

  void testGetWallPolygons(const nav_msgs::msg::Path & segment, WallPolygons & walls)
  {getWallPolygons(segment, walls);}

  void testDrawCorridorWalls(
    nav2_costmap_2d::Costmap2D & g, const std::vector<std::array<double, 2>> & i,
    const std::vector<std::array<double, 2>> & o) {drawCorridorWalls(g, i, o);}

  void testPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {pathCallback(msg);}

  void testResetState() {resetState();}

  rcl_interfaces::msg::SetParametersResult testValidateParams(
    const std::vector<rclcpp::Parameter> & params)
  {return validateParameterUpdatesCallback(params);}

  void testUpdateParams(const std::vector<rclcpp::Parameter> & params)
  {updateParametersCallback(params);}

  void testSaveCorridorInterior(nav2_costmap_2d::Costmap2D & master_grid, const WallPolygons & walls)
  {saveCorridorInterior(master_grid, walls);}

  void testMarkCircleAsInterior(nav2_costmap_2d::Costmap2D & master_grid, int cx, int cy, int r_sq)
  {markCircleAsInterior(master_grid, cx, cy, r_sq);}

  void testFillOutsideCorridor(nav2_costmap_2d::Costmap2D & g, int a, int b, int c, int d)
  {fillOutsideCorridor(g, a, b, c, d);}

  void resetCorridorInteriorMask()
  {std::fill(corridor_interior_mask_.begin(), corridor_interior_mask_.end(), false);}

  bool isInterior(unsigned int flat_idx) const
  {return flat_idx < corridor_interior_mask_.size() && corridor_interior_mask_[flat_idx];}

  bool & enabledRef() {return enabled_;}
  std::atomic_bool & currentRef() {return current_;}
  std::atomic<uint32_t> & pathIndexRef() {return current_path_index_;}

  void setStepSize(size_t s) {step_size_ = s;}
  void setLookAhead(double v) {look_ahead_ = v;}
  void setCorridorWidth(double v) {corridor_width_ = v;}
  void setWallThickness(int v) {wall_thickness_ = v;}
  void setCorridorCost(unsigned char v) {corridor_cost_ = v;}
  void setResolution(double v) {resolution_ = v;}
  void setCostWriteMode(int v) {cost_write_mode_ = v;}
  void setRobotBaseFrame(const std::string & f) {robot_base_frame_ = f;}

  size_t getStepSize() const {return step_size_;}
  double getLookAhead() const {return look_ahead_;}
  double getCorridorWidth() const {return corridor_width_;}
  int getWallThickness() const {return wall_thickness_;}
  unsigned char getCorridorCost() const {return corridor_cost_;}
  int getCostWriteMode() const {return cost_write_mode_;}
  double getResolution() const {return resolution_;}
  const std::string & getCostmapFrame() const {return costmap_frame_;}
};

static constexpr const char * kRobotBaseFrame = "base_link";

static void prepareForUpdateCosts(
  TestableBoundedTrackingErrorLayer * layer,
  nav2_costmap_2d::Costmap2D * costmap)
{
  layer->matchSize();
  layer->enabledRef() = true;
  layer->setRobotBaseFrame(kRobotBaseFrame);
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
}

static geometry_msgs::msg::PoseStamped makePose(
  double x, double y, const std::string & frame = "map")
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;
  ps.pose.orientation.w = 1.0;
  return ps;
}

static nav_msgs::msg::Path makeStraightPath(
  double x0, double y0, double dir_x, double dir_y,
  size_t num_poses, double spacing, const std::string & frame = "map")
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  double len = std::hypot(dir_x, dir_y);
  double ux = dir_x / len, uy = dir_y / len;
  for (size_t i = 0; i < num_poses; ++i) {
    path.poses.push_back(makePose(x0 + ux * spacing * i, y0 + uy * spacing * i, frame));
  }
  return path;
}

static std::shared_ptr<nav_msgs::msg::Path> makeSharedPath(
  double x0, double y0, double dx, double dy,
  int n, double spacing, const rclcpp::Time & stamp,
  const std::string & frame = "map")
{
  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = frame;
  msg->header.stamp = stamp;
  double len = std::hypot(dx, dy);
  double ux = dx / len, uy = dy / len;
  for (int i = 0; i < n; ++i) {
    msg->poses.push_back(makePose(x0 + ux * spacing * i, y0 + uy * spacing * i, frame));
  }
  return msg;
}

class BoundedTrackingErrorLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<TestLifecycleNode>("bte_test_node");
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layers_->resizeMap(100, 100, 0.05, 0.0, 0.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_->now();
    t.header.frame_id = "map";
    t.child_frame_id = kRobotBaseFrame;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    layer_ = std::make_shared<TestableBoundedTrackingErrorLayer>();
    layer_->initialize(layers_.get(), "bte_layer", tf_buffer_.get(), node_, nullptr);
  }

  void TearDown() override
  {
    layer_->deactivate();
    layer_.reset();
    layers_.reset();
  }

  std::shared_ptr<TestLifecycleNode> node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<TestableBoundedTrackingErrorLayer> layer_;

  void setCorridorParams(
    size_t step, double width, int thickness, unsigned char cost, int mode = 0)
  {
    layer_->setStepSize(step);
    layer_->setCorridorWidth(width);
    layer_->setWallThickness(thickness);
    layer_->setCorridorCost(cost);
    layer_->setCostWriteMode(mode);
  }
};

TEST(WallPolygonsTest, testClearAndReserve)
{
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  walls.left_inner.push_back({1.0, 2.0});
  walls.right_outer.push_back({7.0, 8.0});
  walls.clearAndReserve(10);
  EXPECT_TRUE(walls.left_inner.empty());
  EXPECT_TRUE(walls.right_outer.empty());
  EXPECT_GE(walls.left_inner.capacity(), 10u);
}

TEST_F(BoundedTrackingErrorLayerTest, testDefaultParameterValues)
{
  EXPECT_EQ(layer_->getStepSize(), 10u);
  EXPECT_DOUBLE_EQ(layer_->getLookAhead(), 2.5);
  EXPECT_DOUBLE_EQ(layer_->getCorridorWidth(), 2.0);
  EXPECT_EQ(layer_->getWallThickness(), 1);
  EXPECT_EQ(layer_->getCorridorCost(), 190);
  EXPECT_EQ(layer_->getCostWriteMode(), 0);
  EXPECT_TRUE(layer_->enabledRef());
  EXPECT_TRUE(layer_->isClearable());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetPathSegment)
{
  nav_msgs::msg::Path segment;

  layer_->testGetPathSegment(nav_msgs::msg::Path{}, 0, segment);
  EXPECT_TRUE(segment.poses.empty());

  auto path = makeStraightPath(0, 0, 1, 0, 5, 0.1);
  layer_->testGetPathSegment(path, 100, segment);
  EXPECT_TRUE(segment.poses.empty());

  layer_->testGetPathSegment(path, 4, segment);
  ASSERT_EQ(segment.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(segment.poses[0].pose.position.x, path.poses[4].pose.position.x);

  // Boundary pose at look_ahead distance must be included.
  layer_->setLookAhead(0.3);
  auto path2 = makeStraightPath(0, 0, 1, 0, 10, 0.1);
  layer_->testGetPathSegment(path2, 0, segment);
  ASSERT_EQ(segment.poses.size(), 4u);
  EXPECT_NEAR(segment.poses.front().pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(segment.poses.back().pose.position.x, 0.3, 1e-6);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsEdgeCases)
{
  setCorridorParams(1, 2.0, 1, 190);
  layer_->setResolution(0.05);

  TestableBoundedTrackingErrorLayer::WallPolygons walls;

  layer_->testGetWallPolygons(nav_msgs::msg::Path{}, walls);
  EXPECT_TRUE(walls.left_inner.empty());

  nav_msgs::msg::Path single;
  single.header.frame_id = "map";
  single.poses.push_back(makePose(1.0, 1.0));
  layer_->testGetWallPolygons(single, walls);
  EXPECT_TRUE(walls.left_inner.empty());

  single.poses.push_back(makePose(1.0, 1.0));
  layer_->testGetWallPolygons(single, walls);
  EXPECT_TRUE(walls.left_inner.empty());

  // Degenerate first step followed by valid step — valid step must still emit
  layer_->setCorridorWidth(0.5);
  nav_msgs::msg::Path mixed;
  mixed.header.frame_id = "map";
  mixed.poses.push_back(makePose(1.0, 1.0));
  mixed.poses.push_back(makePose(1.0, 1.0));
  mixed.poses.push_back(makePose(1.5, 1.0));
  layer_->testGetWallPolygons(mixed, walls);
  EXPECT_GE(walls.left_inner.size(), 1u);
  EXPECT_EQ(walls.left_inner.size(), walls.right_inner.size());
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsDiagonalOffsets)
{
  // Diagonal path (dx=dy=1): perpendicular is (-1/sqrt2, 1/sqrt2).
  setCorridorParams(1, 2.0, 3, 190);
  layer_->setResolution(0.05);

  auto segment = makeStraightPath(0, 0, 1, 1, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);

  ASSERT_GT(walls.left_inner.size(), 0u);
  const double inner_offset = 1.0;
  const double outer_offset = 1.0 + 3 * 0.05;
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);

  EXPECT_NEAR(walls.left_inner[0][0], -inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.left_inner[0][1], inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.left_outer[0][0], -inv_sqrt2 * outer_offset, 1e-6);
  EXPECT_NEAR(walls.left_outer[0][1], inv_sqrt2 * outer_offset, 1e-6);
  EXPECT_NEAR(walls.right_inner[0][0], inv_sqrt2 * inner_offset, 1e-6);
  EXPECT_NEAR(walls.right_inner[0][1], -inv_sqrt2 * inner_offset, 1e-6);
}

TEST_F(BoundedTrackingErrorLayerTest, testGetWallPolygonsStepSizeSkipsPoses)
{
  setCorridorParams(5, 2.0, 1, 190);
  layer_->setResolution(0.05);

  const size_t expected_points = (21 - 1) / 5;
  auto segment = makeStraightPath(0, 0, 1, 0, 21, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  EXPECT_EQ(walls.left_inner.size(), expected_points);
}



TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsMarksWallsAndPreservesCentre)
{
  layer_->matchSize();
  setCorridorParams(1, 0.5, 2, 190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(2.0, 2.80, mx, my));
  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto segment = makeStraightPath(1.0, 2.5, 1, 0, 21, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls;
  layer_->testGetWallPolygons(segment, walls);
  layer_->testDrawCorridorWalls(*costmap, walls.left_inner, walls.left_outer);
  layer_->testDrawCorridorWalls(*costmap, walls.right_inner, walls.right_outer);

  const double path_y = 2.5;
  const double inner_off = 0.25, outer_off = 0.35;
  const double wall_mid = (inner_off + outer_off) * 0.5;

  ASSERT_TRUE(costmap->worldToMap(2.0, path_y + wall_mid, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190);

  ASSERT_TRUE(costmap->worldToMap(2.0, path_y - wall_mid, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190);

  ASSERT_TRUE(costmap->worldToMap(2.0, path_y, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);

  ASSERT_TRUE(costmap->worldToMap(2.0, 2.80, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(BoundedTrackingErrorLayerTest, testDrawCorridorWallsEdgeCases)
{
  layer_->matchSize();
  layer_->setCorridorCost(190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

  auto isAnyMarked = [&]() {
    for (unsigned int y = 0; y < costmap->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
        if (costmap->getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {return true;}
      }
    }
    return false;
  };

  layer_->testDrawCorridorWalls(*costmap, {{1.0, 1.0}}, {{1.0, 1.1}});
  EXPECT_FALSE(isAnyMarked());

  layer_->testDrawCorridorWalls(
    *costmap, {{10.0, 10.0}, {11.0, 10.0}}, {{10.0, 10.5}, {11.0, 10.5}});
  EXPECT_FALSE(isAnyMarked());

  layer_->testDrawCorridorWalls(
    *costmap, {{1.0, 0.05}, {2.0, 0.05}}, {{1.0, 0.20}, {2.0, 0.20}});
  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(1.5, 0.10, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190);
}

TEST_F(BoundedTrackingErrorLayerTest, testPathCallbackStampBehavior)
{
  auto stamp = node_->now();
  layer_->pathIndexRef().store(42);

  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 1, 0.1, stamp));
  EXPECT_EQ(layer_->pathIndexRef().load(), 0u);

  layer_->pathIndexRef().store(15);
  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 1, 0.1, stamp));
  EXPECT_EQ(layer_->pathIndexRef().load(), 15u);
}

TEST_F(BoundedTrackingErrorLayerTest, testResetAndResetState)
{
  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 1, 0.1, rclcpp::Time{}));
  layer_->pathIndexRef().store(42);

  layer_->currentRef() = true;
  layer_->testResetState();
  EXPECT_EQ(layer_->pathIndexRef().load(), 0u);
  EXPECT_TRUE(layer_->currentRef());

  layer_->pathIndexRef().store(42);
  layer_->reset();
  EXPECT_EQ(layer_->pathIndexRef().load(), 0u);
  EXPECT_FALSE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testValidateParamsRejectsInvalidValues)
{
  layer_->activate();

  EXPECT_FALSE(layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.look_ahead", -1.0)}).successful);
  EXPECT_FALSE(layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.look_ahead", 0.0)}).successful);
  EXPECT_FALSE(layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_width", 0.0)}).successful);

  EXPECT_TRUE(layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.look_ahead", 1.0)}).successful);

  EXPECT_TRUE(layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.step", 0)}).successful);
  EXPECT_TRUE(layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.corridor_cost", 255)}).successful);
  EXPECT_TRUE(layer_->testValidateParams(
    {rclcpp::Parameter("bte_layer.cost_write_mode", -1)}).successful);

  EXPECT_TRUE(layer_->testValidateParams(
    {rclcpp::Parameter("some_other_layer.look_ahead", -99.0)}).successful);
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateParamsApplyAndCurrentReset)
{
  layer_->activate();

  auto applyAndCheck = [&](auto param, auto getter, auto expected) {
    layer_->currentRef() = true;
    layer_->testUpdateParams({param});
    EXPECT_EQ(getter(), expected);
    EXPECT_FALSE(layer_->currentRef());
  };

  applyAndCheck(
    rclcpp::Parameter("bte_layer.look_ahead", 5.0),
    [&]() { return layer_->getLookAhead(); }, 5.0);
  applyAndCheck(
    rclcpp::Parameter("bte_layer.corridor_width", 3.0),
    [&]() { return layer_->getCorridorWidth(); }, 3.0);
  applyAndCheck(
    rclcpp::Parameter("bte_layer.step", 20),
    [&]() { return layer_->getStepSize(); }, size_t{20});
  applyAndCheck(
    rclcpp::Parameter("bte_layer.corridor_cost", 250),
    [&]() { return layer_->getCorridorCost(); }, static_cast<unsigned char>(250));
  applyAndCheck(
    rclcpp::Parameter("bte_layer.wall_thickness", 3),
    [&]() { return layer_->getWallThickness(); }, 3);
  applyAndCheck(
    rclcpp::Parameter("bte_layer.cost_write_mode", 2),
    [&]() { return layer_->getCostWriteMode(); }, 2);

  layer_->currentRef() = true;
  layer_->enabledRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.enabled", false)});
  EXPECT_FALSE(layer_->enabledRef());
  EXPECT_FALSE(layer_->currentRef());

  // Same value must not reset current_
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("bte_layer.look_ahead", layer_->getLookAhead())});
  EXPECT_TRUE(layer_->currentRef());

  layer_->currentRef() = true;
  layer_->testUpdateParams(
    {rclcpp::Parameter("bte_layer.cost_write_mode", layer_->getCostWriteMode())});
  EXPECT_TRUE(layer_->currentRef());

  // Unrelated prefix must be ignored
  layer_->currentRef() = true;
  layer_->testUpdateParams({rclcpp::Parameter("other_layer.look_ahead", 99.0)});
  EXPECT_TRUE(layer_->currentRef());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateBoundsAndMatchSize)
{
  layer_->matchSize();
  EXPECT_DOUBLE_EQ(layer_->getResolution(), 0.05);
  EXPECT_EQ(layer_->getCostmapFrame(), "map");

  layer_->enabledRef() = true;
  double min_x = 1e10, min_y = 1e10, max_x = -1e10, max_y = -1e10;
  layer_->updateBounds(2.5, 2.5, 0.0, &min_x, &min_y, &max_x, &max_y);
  double margin = layer_->getLookAhead() +
    layer_->getCorridorWidth() * 0.5 + layer_->getWallThickness() * layer_->getResolution();
  EXPECT_DOUBLE_EQ(min_x, 2.5 - margin);
  EXPECT_DOUBLE_EQ(max_x, 2.5 + margin);

  // Disabled must not expand
  layer_->enabledRef() = false;
  double bx = 0.0, by = 0.0, ex = 1.0, ey = 1.0;
  layer_->updateBounds(0.5, 0.5, 0.0, &bx, &by, &ex, &ey);
  EXPECT_DOUBLE_EQ(bx, 0.0);
  EXPECT_DOUBLE_EQ(ex, 1.0);
}

TEST_F(BoundedTrackingErrorLayerTest, testDeactivateAndReactivate)
{
  layer_->activate();
  layer_->deactivate();
  EXPECT_NO_THROW(layer_->activate());
}

TEST_F(BoundedTrackingErrorLayerTest, testSaveCorridorInteriorAndAccumulate)
{
  layer_->matchSize();
  setCorridorParams(1, 0.6, 1, 190);
  layer_->setResolution(0.05);

  auto * costmap = layers_->getCostmap();

  auto seg1 = makeStraightPath(1.0, 2.5, 1, 0, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls1;
  layer_->testGetWallPolygons(seg1, walls1);
  layer_->resetCorridorInteriorMask();
  layer_->testSaveCorridorInterior(*costmap, walls1);

  unsigned int cx1, cy1;
  ASSERT_TRUE(costmap->worldToMap(1.5, 2.5, cx1, cy1));
  const unsigned int flat1 = cy1 * costmap->getSizeInCellsX() + cx1;
  EXPECT_TRUE(layer_->isInterior(flat1));

  auto seg2 = makeStraightPath(1.0, 3.5, 1, 0, 11, 0.1);
  TestableBoundedTrackingErrorLayer::WallPolygons walls2;
  layer_->testGetWallPolygons(seg2, walls2);
  layer_->testSaveCorridorInterior(*costmap, walls2);

  unsigned int cx2, cy2;
  ASSERT_TRUE(costmap->worldToMap(1.5, 3.5, cx2, cy2));
  EXPECT_TRUE(layer_->isInterior(cy2 * costmap->getSizeInCellsX() + cx2));
  EXPECT_TRUE(layer_->isInterior(flat1));
}

TEST_F(BoundedTrackingErrorLayerTest, testMarkCircleAsInterior)
{
  layer_->matchSize();
  auto * costmap = layers_->getCostmap();
  const int cx = 50, cy = 50, r = 3;
  layer_->testMarkCircleAsInterior(*costmap, cx, cy, r * r);

  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy <= r * r) {
        unsigned int flat = static_cast<unsigned int>(cy + dy) *
          costmap->getSizeInCellsX() + static_cast<unsigned int>(cx + dx);
        EXPECT_TRUE(layer_->isInterior(flat)) << "Cell (" << cx + dx << "," << cy + dy << ")";
      }
    }
  }
  unsigned int flat_out = static_cast<unsigned int>(cy + r + 1) *
    costmap->getSizeInCellsX() + static_cast<unsigned int>(cx);
  EXPECT_FALSE(layer_->isInterior(flat_out));
}

TEST_F(BoundedTrackingErrorLayerTest, testFillOutsideCorridorBehavior)
{
  layer_->matchSize();
  layer_->setCorridorCost(190);
  auto * costmap = layers_->getCostmap();
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

  layer_->testMarkCircleAsInterior(*costmap, 50, 50, 0);
  costmap->setCost(48, 48, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->testFillOutsideCorridor(*costmap, 47, 47, 52, 52);

  EXPECT_EQ(costmap->getCost(50, 50), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costmap->getCost(51, 50), 190);
  EXPECT_EQ(costmap->getCost(48, 48), nav2_costmap_2d::LETHAL_OBSTACLE);

  // mode 2 overwrites LETHAL outside corridor
  costmap->resetMapToValue(
    0, 0, costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
  layer_->resetCorridorInteriorMask();
  layer_->setCostWriteMode(2);
  costmap->setCost(48, 48, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->testFillOutsideCorridor(*costmap, 47, 47, 52, 52);
  EXPECT_EQ(costmap->getCost(48, 48), 190);
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsEarlyReturns)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);

  auto isClean = [&]() {
    for (unsigned int y = 0; y < costmap->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
        if (costmap->getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {return false;}
      }
    }
    return true;
  };

  auto now = node_->now();
  layer_->enabledRef() = false;
  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 30, 0.1, now));
  layer_->updateCosts(*costmap, 0, 0, 100, 100);
  EXPECT_TRUE(isClean());

  layer_->enabledRef() = true;
  layer_->updateCosts(*costmap, 0, 0, 100, 100);
  EXPECT_TRUE(isClean());

  // stale path (5.1 s > 5.0 s threshold) must reset index
  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 30, 0.1,
    now - rclcpp::Duration::from_seconds(5.1)));
  layer_->pathIndexRef().store(7);
  layer_->updateCosts(*costmap, 0, 0, 100, 100);
  EXPECT_TRUE(isClean());
  EXPECT_EQ(layer_->pathIndexRef().load(), 0u);

  layer_->setResolution(0.0);
  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 30, 0.1, now));
  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));
  EXPECT_TRUE(isClean());

  // step=5 requires min_poses=11, path has only 5
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->setStepSize(5);
  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 5, 0.1, now));
  layer_->updateCosts(*costmap, 0, 0, 100, 100);
  EXPECT_TRUE(isClean());
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsCorridorMode)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  setCorridorParams(1, 0.5, 2, 190);

  layer_->testPathCallback([&]() {
    auto p = std::make_shared<nav_msgs::msg::Path>();
    p->header.frame_id = "map";
    p->header.stamp = node_->now() - rclcpp::Duration::from_seconds(4.9);
    for (int i = 0; i < 30; ++i) {p->poses.push_back(makePose(i * 0.1, 2.5));}
    return p;
  }());
  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(1.5, 2.80, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190);

  // out-of-range path index resets silently to 0
  prepareForUpdateCosts(layer_.get(), costmap);
  layer_->testPathCallback(makeSharedPath(0, 0, 1, 0, 10, 0.1, node_->now()));
  layer_->pathIndexRef().store(999);
  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));
  EXPECT_EQ(layer_->pathIndexRef().load(), 0u);
}

TEST_F(BoundedTrackingErrorLayerTest, testUpdateCostsFillMode)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  setCorridorParams(1, 0.5, 1, 190, 1);
  layer_->setLookAhead(2.5);

  layer_->testPathCallback([&]() {
    auto p = std::make_shared<nav_msgs::msg::Path>();
    p->header.frame_id = "map";
    p->header.stamp = node_->now();
    for (int i = 0; i < 60; ++i) {p->poses.push_back(makePose(i * 0.05, 2.5));}
    return p;
  }());
  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(0.5, 0.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190);

  ASSERT_TRUE(costmap->worldToMap(0.5, 2.5, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);

  prepareForUpdateCosts(layer_.get(), costmap);
  setCorridorParams(1, 1.0, 1, 190, 1);
  layer_->testPathCallback(makeSharedPath(1.0, 0.0, 1, 0, 40, 0.1, node_->now()));
  layer_->updateCosts(*costmap, 0, 0, 100, 100);

  unsigned int rx, ry;
  ASSERT_TRUE(costmap->worldToMap(0.0, 0.0, rx, ry));
  EXPECT_TRUE(layer_->isInterior(ry * costmap->getSizeInCellsX() + rx));

  unsigned int fx, fy;
  ASSERT_TRUE(costmap->worldToMap(1.0, 0.0, fx, fy));
  EXPECT_FALSE(layer_->isInterior(fy * costmap->getSizeInCellsX() + fx));
}

TEST_F(BoundedTrackingErrorLayerTest, testFillBranchPathExitsAndReentersBbox)
{
  auto * costmap = layers_->getCostmap();
  prepareForUpdateCosts(layer_.get(), costmap);
  setCorridorParams(1, 0.4, 1, 190, 1);
  layer_->setLookAhead(1.0);

  auto msg = std::make_shared<nav_msgs::msg::Path>();
  msg->header.frame_id = "map";
  msg->header.stamp = node_->now();
  for (int i = 0; i <= 10; ++i) {msg->poses.push_back(makePose(i * 0.1, 0.3));}   // chunk A
  for (int i = 13; i <= 20; ++i) {msg->poses.push_back(makePose(i * 0.1, 0.3));}  // gap
  for (int i = 10; i >= 0; --i) {msg->poses.push_back(makePose(i * 0.1, 0.7));}   // chunk B

  layer_->testPathCallback(msg);
  ASSERT_NO_THROW(layer_->updateCosts(*costmap, 0, 0, 100, 100));

  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(0.5, 0.3, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);

  ASSERT_TRUE(costmap->worldToMap(0.5, 0.7, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);

  ASSERT_TRUE(costmap->worldToMap(0.5, 1.1, mx, my));
  EXPECT_EQ(costmap->getCost(mx, my), 190);
}
