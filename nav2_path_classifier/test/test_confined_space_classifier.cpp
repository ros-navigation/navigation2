// Copyright (c) 2026 Origin
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
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_path_classifier/classifiers/confined_space_classifier.hpp"

namespace nav2_path_classifier
{

/**
 * @class DummyCostmapSubscriber
 * @brief Costmap subscriber whose costmap is set directly by the test.
 */
class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  /**
   * @brief A constructor for DummyCostmapSubscriber
   * @param node Node that owns the subscriber
   * @param topic_name Costmap topic, never published to by the test
   */
  DummyCostmapSubscriber(
    const nav2_util::LifecycleNode::WeakPtr & node, const std::string & topic_name)
  : CostmapSubscriber(node, topic_name)
  {
  }

  /**
   * @brief Set the costmap returned by getCostmap().
   * @param msg Costmap message
   */
  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    costmap_msg_ = msg;
    costmap_received_ = true;
  }
};

/**
 * @class DummyFootprintSubscriber
 * @brief Footprint subscriber whose footprint is set directly by the test.
 */
class DummyFootprintSubscriber : public nav2_costmap_2d::FootprintSubscriber
{
public:
  /**
   * @brief A constructor for DummyFootprintSubscriber
   * @param node Node that owns the subscriber
   * @param topic_name Footprint topic, never published to by the test
   * @param tf TF buffer used to look up the footprint in the robot frame
   */
  DummyFootprintSubscriber(
    const nav2_util::LifecycleNode::WeakPtr & node, const std::string & topic_name,
    tf2_ros::Buffer & tf)
  : FootprintSubscriber(node, topic_name, tf)
  {
  }

  /**
   * @brief Set a square footprint in the robot frame.
   * @param half_side Half the side length of the square in metres
   */
  void setSquareFootprint(double half_side)
  {
    setFootprint(
      {{half_side, half_side}, {-half_side, half_side},
        {-half_side, -half_side}, {half_side, -half_side}});
  }

  /**
   * @brief Set a footprint in the robot frame from a list of (x, y) vertices.
   * @param vertices Footprint vertices
   */
  void setFootprint(const std::vector<std::pair<double, double>> & vertices)
  {
    auto msg = std::make_shared<geometry_msgs::msg::PolygonStamped>();
    msg->header.frame_id = "base_link";
    for (const auto & vertex : vertices) {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(vertex.first);
      point.y = static_cast<float>(vertex.second);
      msg->polygon.points.push_back(point);
    }
    footprint_ = msg;
    footprint_received_ = true;
  }
};

/**
 * @brief Build a 10 m x 10 m free costmap centered on the origin at 0.05 m resolution.
 * @return Costmap message
 */
static nav2_msgs::msg::Costmap::SharedPtr makeFreeCostmap()
{
  auto msg = std::make_shared<nav2_msgs::msg::Costmap>();
  msg->header.frame_id = "map";
  msg->metadata.size_x = 200;
  msg->metadata.size_y = 200;
  msg->metadata.resolution = 0.05;
  msg->metadata.origin.position.x = -5.0;
  msg->metadata.origin.position.y = -5.0;
  msg->data.resize(msg->metadata.size_x * msg->metadata.size_y, nav2_costmap_2d::FREE_SPACE);
  return msg;
}

/**
 * @brief Mark every cell whose center y lies in [y_min, y_max) as lethal, across the full width.
 * @param msg Costmap message to modify
 * @param y_min Lower y bound of the wall in metres
 * @param y_max Upper y bound of the wall in metres
 */
static void addWall(nav2_msgs::msg::Costmap & msg, double y_min, double y_max)
{
  const auto & meta = msg.metadata;
  for (unsigned int row = 0; row < meta.size_y; ++row) {
    const double y = meta.origin.position.y + (row + 0.5) * meta.resolution;
    if (y < y_min || y >= y_max) {
      continue;
    }
    for (unsigned int col = 0; col < meta.size_x; ++col) {
      msg.data[row * meta.size_x + col] = nav2_costmap_2d::LETHAL_OBSTACLE;
    }
  }
}

/**
 * @brief Build a pose in the map frame facing along +x.
 * @param x X position in metres
 * @param y Y position in metres
 * @return Pose
 */
static geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

class ConfinedSpaceClassifierWrapper : public ConfinedSpaceClassifier
{
public:
  nav2_costmap_2d::Footprint inflateFootprint(
    const nav2_costmap_2d::Footprint & fp, double delta) const
  {
    return ConfinedSpaceClassifier::inflateFootprint(fp, delta);
  }

  nav2_costmap_2d::Footprint orientFootprint(
    const nav2_costmap_2d::Footprint & fp,
    double x, double y, double cos_th, double sin_th) const
  {
    return ConfinedSpaceClassifier::orientFootprint(fp, x, y, cos_th, sin_th);
  }

  std::vector<size_t> buildOppositePairs(const nav2_costmap_2d::Footprint & fp) const
  {
    return ConfinedSpaceClassifier::buildOppositePairs(fp);
  }
};

static geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

// Square footprint centered at the origin
static nav2_costmap_2d::Footprint makeSquare(double half_side)
{
  return {
    makePoint(half_side, half_side),
    makePoint(-half_side, half_side),
    makePoint(-half_side, -half_side),
    makePoint(half_side, -half_side)
  };
}

// Rectangle footprint: front=0.45, rear=-0.65, left=0.375, right=-0.375
static nav2_costmap_2d::Footprint makeRectangle()
{
  return {
    makePoint(0.45, 0.375),
    makePoint(-0.65, 0.375),
    makePoint(-0.65, -0.375),
    makePoint(0.45, -0.375)
  };
}

static void getBounds(
  const nav2_costmap_2d::Footprint & fp,
  double & min_x, double & max_x, double & min_y, double & max_y)
{
  min_x = min_y = 1e18;
  max_x = max_y = -1e18;
  for (const auto & pt : fp) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }
}

class ConfinedSpaceClassifierTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    node_ = std::make_shared<nav2_util::LifecycleNode>("confined_space_classifier_test");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    // Register base_link so looking up the footprint in its own frame resolves immediately
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    tf_->setTransform(transform, "test", true);

    costmap_sub_ = std::make_shared<DummyCostmapSubscriber>(node_, "costmap");
    footprint_sub_ = std::make_shared<DummyFootprintSubscriber>(node_, "footprint", *tf_);

    node_->declare_parameter("confined.class_type", 2);
    node_->declare_parameter("confined.inflation_resolution", 0.1);
    node_->declare_parameter("confined.max_constraint_clearance", 0.5);
  }

  void TearDown() override
  {
    footprint_sub_.reset();
    costmap_sub_.reset();
    tf_.reset();
    node_.reset();
  }

  /**
   * @brief Configure the classifier against the test node and dummy subscribers.
   */
  void configure()
  {
    classifier_.configure(node_, "confined", tf_, costmap_sub_, footprint_sub_);
  }

  /**
   * @brief Classify a single pose path and report whether the pose is confined.
   * @param x X position in metres
   * @param y Y position in metres
   * @return true if the pose was classified as confined space
   */
  bool isConfinedAt(double x, double y)
  {
    nav_msgs::msg::Path path;
    path.poses.push_back(makePose(x, y));
    return classifier_.classify(path)[0].class_type != nav2_msgs::msg::ClassifiedPose::UNKNOWN;
  }

  /**
   * @brief Set a costmap with lethal walls on both sides of the x axis.
   * @param inner Distance from the x axis to the inner face of each wall in metres
   * @param outer Distance from the x axis to the outer face of each wall in metres
   */
  void setCorridor(double inner, double outer)
  {
    auto costmap = makeFreeCostmap();
    addWall(*costmap, inner, outer);
    addWall(*costmap, -outer, -inner);
    costmap_sub_->setCostmap(costmap);
  }

  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<DummyCostmapSubscriber> costmap_sub_;
  std::shared_ptr<DummyFootprintSubscriber> footprint_sub_;
  ConfinedSpaceClassifierWrapper classifier_;
};

TEST_F(ConfinedSpaceClassifierTest, ConfigureRejectsNonPositiveResolution)
{
  node_->set_parameter(rclcpp::Parameter("confined.inflation_resolution", 0.0));
  EXPECT_THROW(configure(), std::runtime_error);
}

TEST_F(ConfinedSpaceClassifierTest, ConfigureRequiresClassType)
{
  EXPECT_ANY_THROW(
    classifier_.configure(node_, "missing", tf_, costmap_sub_, footprint_sub_));
}

TEST_F(ConfinedSpaceClassifierTest, ConfigureThrowsOnExpiredParent)
{
  rclcpp_lifecycle::LifecycleNode::WeakPtr expired;
  EXPECT_THROW(
    classifier_.configure(expired, "confined", tf_, costmap_sub_, footprint_sub_),
    std::runtime_error);
}

TEST_F(ConfinedSpaceClassifierTest, NoMatchWithoutFootprint)
{
  configure();
  setCorridor(0.55, 0.85);
  EXPECT_FALSE(isConfinedAt(0.0, 0.0));
}

TEST_F(ConfinedSpaceClassifierTest, ThrowsWithoutCostmap)
{
  configure();
  footprint_sub_->setSquareFootprint(0.3);
  EXPECT_THROW(isConfinedAt(0.0, 0.0), std::runtime_error);
}

TEST_F(ConfinedSpaceClassifierTest, NoMatchForDegenerateFootprint)
{
  configure();
  setCorridor(0.55, 0.85);
  footprint_sub_->setFootprint({{0.3, 0.3}, {-0.3, -0.3}});
  EXPECT_FALSE(isConfinedAt(0.0, 0.0));
}

TEST_F(ConfinedSpaceClassifierTest, NoMatchWhenWallsBeyondClearance)
{
  configure();
  // Footprint edge reaches 0.3 + 0.5 = 0.8 m at most, short of walls starting at 1.0 m
  setCorridor(1.0, 1.3);
  footprint_sub_->setSquareFootprint(0.3);
  EXPECT_FALSE(isConfinedAt(0.0, 0.0));
}

TEST_F(ConfinedSpaceClassifierTest, ClassifiesEachPoseOfPath)
{
  node_->set_parameter(rclcpp::Parameter("confined.class_type", 7));
  configure();
  setCorridor(0.55, 0.85);
  footprint_sub_->setSquareFootprint(0.3);

  // First pose is in the corridor, second is above the top wall in open space
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(0.0, 3.0));
  const auto classified_path = classifier_.classify(path);

  ASSERT_EQ(classified_path.size(), 2u);
  EXPECT_EQ(classified_path[0].class_type, 7u);
  EXPECT_EQ(classified_path[1].class_type, nav2_msgs::msg::ClassifiedPose::UNKNOWN);
  EXPECT_DOUBLE_EQ(classified_path[1].pose.pose.position.y, 3.0);
}

TEST_F(ConfinedSpaceClassifierTest, EmptyPathGivesEmptyResult)
{
  configure();
  EXPECT_TRUE(classifier_.classify(nav_msgs::msg::Path()).empty());
}

TEST_F(ConfinedSpaceClassifierTest, NewCostmapPickedUpOnNextPath)
{
  configure();
  setCorridor(0.55, 0.85);
  footprint_sub_->setSquareFootprint(0.3);
  EXPECT_TRUE(isConfinedAt(0.0, 0.0));

  costmap_sub_->setCostmap(makeFreeCostmap());
  EXPECT_FALSE(isConfinedAt(0.0, 0.0));
}

TEST_F(ConfinedSpaceClassifierTest, FootprintChangeIsPickedUp)
{
  configure();
  setCorridor(0.75, 1.0);

  // Edge reaches 0.3 + 0.5 = 0.8 m, inside walls starting at 0.75 m
  footprint_sub_->setSquareFootprint(0.3);
  EXPECT_TRUE(isConfinedAt(0.0, 0.0));

  // Edge reaches 0.1 + 0.5 = 0.6 m, short of the walls
  footprint_sub_->setSquareFootprint(0.1);
  EXPECT_FALSE(isConfinedAt(0.0, 0.0));
}

TEST_F(ConfinedSpaceClassifierTest, InflateSquareGrowsByDelta)
{
  auto fp = makeSquare(0.5);
  const double delta = 0.2;
  auto inflated = classifier_.inflateFootprint(fp, delta);

  // Mitered offset keeps the vertex count for convex polygons, which isConfined() relies on
  EXPECT_EQ(inflated.size(), fp.size());

  double min_x, max_x, min_y, max_y;
  getBounds(inflated, min_x, max_x, min_y, max_y);
  EXPECT_NEAR(max_x, 0.5 + delta, 0.01);
  EXPECT_NEAR(-min_x, 0.5 + delta, 0.01);
  EXPECT_NEAR(max_y, 0.5 + delta, 0.01);
  EXPECT_NEAR(-min_y, 0.5 + delta, 0.01);
}

TEST_F(ConfinedSpaceClassifierTest, InflateRectangleGrowsByDelta)
{
  auto fp = makeRectangle();
  const double delta = 0.1;
  auto inflated = classifier_.inflateFootprint(fp, delta);

  double min_x, max_x, min_y, max_y;
  getBounds(inflated, min_x, max_x, min_y, max_y);
  EXPECT_NEAR(max_x, 0.45 + delta, 0.01);
  EXPECT_NEAR(min_x, -0.65 - delta, 0.01);
  EXPECT_NEAR(max_y, 0.375 + delta, 0.01);
  EXPECT_NEAR(min_y, -0.375 - delta, 0.01);
}

TEST_F(ConfinedSpaceClassifierTest, Orient90DegreeRotation)
{
  auto fp = makeSquare(0.5);
  // 90 degrees: cos=0, sin=1
  auto oriented = classifier_.orientFootprint(fp, 0.0, 0.0, 0.0, 1.0);

  // (0.5, 0.5) rotated 90 degrees -> (-0.5, 0.5), (-0.5, 0.5) -> (-0.5, -0.5)
  ASSERT_EQ(oriented.size(), fp.size());
  EXPECT_NEAR(oriented[0].x, -0.5, 1e-9);
  EXPECT_NEAR(oriented[0].y, 0.5, 1e-9);
  EXPECT_NEAR(oriented[1].x, -0.5, 1e-9);
  EXPECT_NEAR(oriented[1].y, -0.5, 1e-9);
}

TEST_F(ConfinedSpaceClassifierTest, OrientTranslationAndRotation)
{
  auto fp = makeSquare(0.5);
  const double tx = 1.0, ty = 2.0;
  const double theta = M_PI / 4.0;
  const double cos_th = std::cos(theta);
  const double sin_th = std::sin(theta);

  auto oriented = classifier_.orientFootprint(fp, tx, ty, cos_th, sin_th);

  ASSERT_EQ(oriented.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    EXPECT_NEAR(oriented[i].x, tx + (fp[i].x * cos_th - fp[i].y * sin_th), 1e-9);
    EXPECT_NEAR(oriented[i].y, ty + (fp[i].x * sin_th + fp[i].y * cos_th), 1e-9);
  }
}

TEST_F(ConfinedSpaceClassifierTest, OppositePairsRectangle)
{
  auto opp = classifier_.buildOppositePairs(makeRectangle());

  // Front <-> rear and left <-> right
  ASSERT_EQ(opp.size(), 4u);
  EXPECT_EQ(opp[0], 2u);
  EXPECT_EQ(opp[2], 0u);
  EXPECT_EQ(opp[1], 3u);
  EXPECT_EQ(opp[3], 1u);
}

TEST_F(ConfinedSpaceClassifierTest, OppositePairsHexagon)
{
  nav2_costmap_2d::Footprint hex;
  for (int i = 0; i < 6; ++i) {
    const double angle = i * M_PI / 3.0;
    hex.push_back(makePoint(std::cos(angle), std::sin(angle)));
  }

  // For a regular hexagon the opposite of edge i is edge (i + 3) % 6
  auto opp = classifier_.buildOppositePairs(hex);
  ASSERT_EQ(opp.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(opp[i], (i + 3) % 6) << "edge " << i;
  }
}

}  // namespace nav2_path_classifier

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
