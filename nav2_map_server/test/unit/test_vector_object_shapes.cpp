// Copyright (c) 2023 Samsung R&D Institute Russia
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
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav2_msgs/msg/polygon_object.hpp"
#include "nav2_msgs/msg/circle_object.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/occ_grid_utils.hpp"

#include "nav2_map_server/vector_object_shapes.hpp"
#include "nav2_map_server/vector_object_utils.hpp"

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

static const char POLYGON_NAME[]{"Polygon"};
static const char CIRCLE_NAME[]{"Circle"};
static const char GLOBAL_FRAME_ID[]{"map"};
static const char SHAPE_FRAME_ID[]{"shape"};
static double FRAME_SHIFT = 0.1;
static std::vector<double> POINTS{1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0};
static std::vector<double> CENTER{0.0, 0.0};

class PolygonWrapper : public nav2_map_server::Polygon
{
public:
  explicit PolygonWrapper(const nav2_util::LifecycleNode::WeakPtr & node)
  : Polygon(node)
  {}

  geometry_msgs::msg::Polygon::SharedPtr getPoly()
  {
    return polygon_;
  }
};  // PolygonWrapper

class CircleWrapper : public nav2_map_server::Circle
{
public:
  explicit CircleWrapper(const nav2_util::LifecycleNode::WeakPtr & node)
  : Circle(node)
  {}

  geometry_msgs::msg::Point32::SharedPtr getCenter()
  {
    return center_;
  }
};  // CircleWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

protected:
  void setPolygonParams(const std::string & uuid);
  void setCircleParams(const std::string & uuid);
  nav2_msgs::msg::PolygonObject::SharedPtr makePolygonObject(
    const std::vector<unsigned char> & uuid);
  nav2_msgs::msg::CircleObject::SharedPtr makeCircleObject(
    const std::vector<unsigned char> & uuid);

  void sendTransform();

  nav_msgs::msg::OccupancyGrid::SharedPtr makeMap();
  void verifyPolygonBorders(nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void verifyCircleBorders(nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void verifyMapEmpty(nav_msgs::msg::OccupancyGrid::SharedPtr map);

  void comparePolygonObjects(
    nav2_msgs::msg::PolygonObject::SharedPtr p1,
    nav2_msgs::msg::PolygonObject::SharedPtr p2);
  void compareCircleObjects(
    nav2_msgs::msg::CircleObject::SharedPtr c1,
    nav2_msgs::msg::CircleObject::SharedPtr c2);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<PolygonWrapper> polygon_;
  std::shared_ptr<CircleWrapper> circle_;

  nav2_util::LifecycleNode::SharedPtr node_;
};  // Tester

Tester::Tester()
{
  node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");

  // Create shapes
  polygon_ = std::make_shared<PolygonWrapper>(node_);
  circle_ = std::make_shared<CircleWrapper>(node_);

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Tester::~Tester()
{
  polygon_.reset();
  circle_.reset();

  node_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();
}

void Tester::setPolygonParams(const std::string & uuid)
{
  const std::string polygon_name(POLYGON_NAME);

  node_->declare_parameter(
    polygon_name + ".type", rclcpp::ParameterValue("polygon"));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".type", "polygon"));

  node_->declare_parameter(
    polygon_name + ".frame_id", rclcpp::ParameterValue(GLOBAL_FRAME_ID));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".frame_id", GLOBAL_FRAME_ID));

  node_->declare_parameter(
    polygon_name + ".closed", rclcpp::ParameterValue(true));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".closed", true));

  node_->declare_parameter(
    polygon_name + ".value", rclcpp::ParameterValue(nav2_util::OCC_GRID_OCCUPIED));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".value", nav2_util::OCC_GRID_OCCUPIED));

  node_->declare_parameter(
    polygon_name + ".points", rclcpp::ParameterValue(POINTS));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".points", POINTS));

  if (!uuid.empty()) {
    node_->declare_parameter(
      polygon_name + ".uuid", rclcpp::ParameterValue(uuid));
    node_->set_parameter(
      rclcpp::Parameter(polygon_name + ".uuid", uuid));
  }
}

void Tester::setCircleParams(const std::string & uuid)
{
  const std::string circle_name(CIRCLE_NAME);

  node_->declare_parameter(
    circle_name + ".type", rclcpp::ParameterValue("circle"));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".type", "circle"));

  node_->declare_parameter(
    circle_name + ".frame_id", rclcpp::ParameterValue(GLOBAL_FRAME_ID));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".frame_id", GLOBAL_FRAME_ID));

  node_->declare_parameter(
    circle_name + ".fill", rclcpp::ParameterValue(true));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".fill", true));

  node_->declare_parameter(
    circle_name + ".value", rclcpp::ParameterValue(nav2_util::OCC_GRID_OCCUPIED));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".value", nav2_util::OCC_GRID_OCCUPIED));

  node_->declare_parameter(
    circle_name + ".center", rclcpp::ParameterValue(CENTER));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".center", CENTER));

  node_->declare_parameter(
    circle_name + ".radius", rclcpp::ParameterValue(1.0));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".radius", 1.0));

  if (!uuid.empty()) {
    node_->declare_parameter(
      circle_name + ".uuid", rclcpp::ParameterValue(uuid));
    node_->set_parameter(
      rclcpp::Parameter(circle_name + ".uuid", uuid));
  }
}

nav2_msgs::msg::PolygonObject::SharedPtr Tester::makePolygonObject(
  const std::vector<unsigned char> & uuid)
{
  nav2_msgs::msg::PolygonObject::SharedPtr po = std::make_shared<nav2_msgs::msg::PolygonObject>();
  po->header.frame_id = GLOBAL_FRAME_ID;
  if (uuid.size() == 16) {
    // uuid could be optionally specified
    std::array<unsigned char, 16> uuid_array;
    std::copy(uuid.begin(), uuid.end(), uuid_array.begin());
    po->uuid.uuid = uuid_array;
  }
  geometry_msgs::msg::Point32 p;
  p.x = 1.0;
  p.y = 1.0;
  po->points.push_back(p);
  p.x = -1.0;
  po->points.push_back(p);
  p.y = -1.0;
  po->points.push_back(p);
  p.x = 1.0;
  po->points.push_back(p);
  po->closed = true;
  po->value = nav2_util::OCC_GRID_OCCUPIED;

  return po;
}

nav2_msgs::msg::CircleObject::SharedPtr Tester::makeCircleObject(
  const std::vector<unsigned char> & uuid)
{
  nav2_msgs::msg::CircleObject::SharedPtr co = std::make_shared<nav2_msgs::msg::CircleObject>();
  co->header.frame_id = GLOBAL_FRAME_ID;
  if (uuid.size() == 16) {
    // uuid could be optionally specified
    std::array<unsigned char, 16> uuid_array;
    std::copy(uuid.begin(), uuid.end(), uuid_array.begin());
    co->uuid.uuid = uuid_array;
  }
  co->center.x = 0.0;
  co->center.y = 0.0;
  co->radius = 1.0;
  co->fill = true;
  co->value = nav2_util::OCC_GRID_OCCUPIED;

  return co;
}

void Tester::sendTransform()
{
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  geometry_msgs::msg::TransformStamped transform;

  // global_frame -> shape_frame transform
  transform.header.frame_id = GLOBAL_FRAME_ID;
  transform.child_frame_id = SHAPE_FRAME_ID;

  transform.header.stamp = node_->now();
  transform.transform.translation.x = FRAME_SHIFT;
  transform.transform.translation.y = FRAME_SHIFT;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster->sendTransform(transform);
}

nav_msgs::msg::OccupancyGrid::SharedPtr Tester::makeMap()
{
  nav_msgs::msg::OccupancyGrid::SharedPtr map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.frame_id = GLOBAL_FRAME_ID;
  map->info.resolution = 0.1;
  map->info.width = 40;
  map->info.height = 40;
  map->info.origin.position.x = -2.0;
  map->info.origin.position.y = -2.0;
  map->data = std::vector<int8_t>(400 * 400, nav2_util::OCC_GRID_FREE);

  return map;
}

void Tester::verifyPolygonBorders(nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  // Map is expected to be as follows:
  // 0 0 0 0 0 0 0 0
  // 0 0 x x x x 0 0
  // 0 0 x 0 0 0 0 0
  // 0 0 x 0 0 0 0 0
  // 0 0 x x x x 0 0
  // 0 0 0 0 0 0 0 0
  const unsigned int map_center_x = 19;
  const unsigned int map_center_y = 19;

  for (unsigned int my = 0; my < map->info.height; my++) {
    for (unsigned int mx = 0; mx < map->info.width; mx++) {
      if (my == map_center_y - 10 && mx >= map_center_x - 10 && mx <= map_center_x + 10) {
        // 1st border
        ASSERT_EQ(map->data[my * map->info.width + mx], nav2_util::OCC_GRID_OCCUPIED);
      } else if (mx == map_center_x - 10 && my >= map_center_y - 10 && my <= map_center_y + 10) {
        // 2nd border
        ASSERT_EQ(map->data[my * map->info.width + mx], nav2_util::OCC_GRID_OCCUPIED);
      } else if (my == map_center_y + 10 && mx >= map_center_x - 10 && mx <= map_center_x + 10) {
        // 3rd border
        ASSERT_EQ(map->data[my * map->info.width + mx], nav2_util::OCC_GRID_OCCUPIED);
      } else {
        // Does not belong to any border
        ASSERT_EQ(map->data[my * map->info.width + mx], nav2_util::OCC_GRID_FREE);
      }
    }
  }
}

void Tester::verifyCircleBorders(nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  // Circle center in cell coordinates
  const double circle_center_x = 19.5;  // 19 cell's origin + 0.5 center of cell
  const double circle_center_y = 19.5;  // 19 cell's origin + 0.5 center of cell

  double radius;
  for (unsigned int my = 0; my < map->info.height; my++) {
    for (unsigned int mx = 0; mx < map->info.width; mx++) {
      if (map->data[my * map->info.width + mx] == nav2_util::OCC_GRID_OCCUPIED) {
        radius = std::hypot(circle_center_x - mx, circle_center_y - my);
        ASSERT_NEAR(radius, 10.0, 1.0);  // Border drift no more than once cell
      }
    }
  }
}

void Tester::verifyMapEmpty(nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  for (unsigned int my = 0; my < map->info.height; my++) {
    for (unsigned int mx = 0; mx < map->info.width; mx++) {
      ASSERT_EQ(map->data[my * map->info.width + mx], nav2_util::OCC_GRID_FREE);
    }
  }
}

void Tester::comparePolygonObjects(
  nav2_msgs::msg::PolygonObject::SharedPtr p1,
  nav2_msgs::msg::PolygonObject::SharedPtr p2)
{
  ASSERT_EQ(p1->header.frame_id, p2->header.frame_id);
  ASSERT_EQ(p1->uuid.uuid, p2->uuid.uuid);
  ASSERT_EQ(p1->points.size(), p2->points.size());
  for (unsigned int i = 0; i < p1->points.size(); i++) {
    ASSERT_NEAR(p1->points[i].x, p2->points[i].x, EPSILON);
    ASSERT_NEAR(p1->points[i].y, p2->points[i].y, EPSILON);
  }
  ASSERT_EQ(p1->closed, p2->closed);
  ASSERT_EQ(p1->value, p2->value);
}

void Tester::compareCircleObjects(
  nav2_msgs::msg::CircleObject::SharedPtr c1,
  nav2_msgs::msg::CircleObject::SharedPtr c2)
{
  ASSERT_EQ(c1->header.frame_id, c2->header.frame_id);
  ASSERT_EQ(c1->uuid.uuid, c2->uuid.uuid);
  ASSERT_NEAR(c1->center.x, c2->center.x, EPSILON);
  ASSERT_NEAR(c1->center.y, c2->center.y, EPSILON);
  ASSERT_NEAR(c1->radius, c2->radius, EPSILON);
  ASSERT_EQ(c1->fill, c2->fill);
  ASSERT_EQ(c1->value, c2->value);
}

//---------- Polygon testcases ----------

TEST_F(Tester, testPolygonObtainParams)
{
  setPolygonParams("01010101-0101-0101-0101-010101010101");
  ASSERT_TRUE(polygon_->obtainParams(POLYGON_NAME));

  ASSERT_EQ(polygon_->getType(), nav2_map_server::POLYGON);
  ASSERT_EQ(polygon_->getValue(), nav2_util::OCC_GRID_OCCUPIED);
  ASSERT_EQ(polygon_->getFrameID(), GLOBAL_FRAME_ID);
  ASSERT_EQ(polygon_->isFill(), true);
  ASSERT_EQ(polygon_->getUUID(), "01010101-0101-0101-0101-010101010101");
}

TEST_F(Tester, testPolygonObtainIncorrectParams)
{
  // Setting polygon parameters w/o points
  const std::string polygon_name(POLYGON_NAME);

  node_->declare_parameter(
    polygon_name + ".type", rclcpp::ParameterValue("polygon"));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".type", "polygon"));

  node_->declare_parameter(
    polygon_name + ".frame_id", rclcpp::ParameterValue(GLOBAL_FRAME_ID));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".frame_id", GLOBAL_FRAME_ID));

  node_->declare_parameter(
    polygon_name + ".closed", rclcpp::ParameterValue(true));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".closed", true));

  node_->declare_parameter(
    polygon_name + ".value", rclcpp::ParameterValue(nav2_util::OCC_GRID_OCCUPIED));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".value", nav2_util::OCC_GRID_OCCUPIED));

  node_->declare_parameter(
    polygon_name + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Check that points is mandatory parameter for the polygon
  ASSERT_FALSE(polygon_->obtainParams(polygon_name));

  // Setting incorrect number of points
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".points", std::vector{1.0, 2.0, 3.0}));
  ASSERT_FALSE(polygon_->obtainParams(polygon_name));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".points", POINTS));

  // Setting incorrect UUID
  node_->declare_parameter(
    polygon_name + ".uuid", rclcpp::ParameterValue("incorrect-uuid"));
  node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".uuid", "incorrect-uuid"));
  ASSERT_FALSE(polygon_->obtainParams(polygon_name));
}

TEST_F(Tester, testPolygonSetParams)
{
  nav2_msgs::msg::PolygonObject::SharedPtr po = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2});

  // Check that parameters were set correctly
  ASSERT_TRUE(polygon_->setParams(po));
  ASSERT_EQ(polygon_->getType(), nav2_map_server::POLYGON);
  ASSERT_EQ(polygon_->getValue(), nav2_util::OCC_GRID_OCCUPIED);
  ASSERT_EQ(polygon_->getFrameID(), GLOBAL_FRAME_ID);
  ASSERT_EQ(polygon_->isFill(), true);
  ASSERT_EQ(polygon_->getUUID(), "01010101-0101-0101-0101-020202020202");

  // Verify that getting the parameters also works correctly
  nav2_msgs::msg::PolygonObject::SharedPtr params = polygon_->getParams();
  comparePolygonObjects(params, po);
}

TEST_F(Tester, testPolygonSetIncorrectParams)
{
  nav2_msgs::msg::PolygonObject::SharedPtr po = makePolygonObject(
    std::vector<unsigned char>());
  // Setting incorrect number of vertices
  po->points.resize(2);

  // And check that it is failed to set these parameters
  ASSERT_FALSE(polygon_->setParams(po));
}

TEST_F(Tester, testPolygonBoundaries)
{
  setPolygonParams("");
  ASSERT_TRUE(polygon_->obtainParams(POLYGON_NAME));

  double min_x, min_y, max_x, max_y;
  polygon_->getBoundaries(min_x, min_y, max_x, max_y);

  ASSERT_NEAR(min_x, -1.0, EPSILON);
  ASSERT_NEAR(min_y, -1.0, EPSILON);
  ASSERT_NEAR(max_x, 1.0, EPSILON);
  ASSERT_NEAR(max_y, 1.0, EPSILON);
}

TEST_F(Tester, testPolygonPoints)
{
  setPolygonParams("");
  ASSERT_TRUE(polygon_->obtainParams(POLYGON_NAME));

  ASSERT_TRUE(polygon_->isPointInside(0.0, 0.0));
  ASSERT_FALSE(polygon_->isPointInside(-2.0, -2.0));
  ASSERT_FALSE(polygon_->isPointInside(2.0, 2.0));
}

TEST_F(Tester, testPolygonBorders)
{
  setPolygonParams("");
  node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".closed", false));
  ASSERT_TRUE(polygon_->obtainParams(POLYGON_NAME));

  nav_msgs::msg::OccupancyGrid::SharedPtr map = makeMap();

  polygon_->putBorders(map, nav2_map_server::OverlayType::OVERLAY_SEQ);

  verifyPolygonBorders(map);
}

TEST_F(Tester, testPolygonBordersOutOfBoundaries)
{
  setPolygonParams("");
  node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".closed", false));
  // Set polygon to be out of map
  node_->set_parameter(
    rclcpp::Parameter(
      std::string(POLYGON_NAME) + ".points",
      std::vector<double>{5.0, 5.0, 6.0, 5.0, 5.0, 6.0}));
  ASSERT_TRUE(polygon_->obtainParams(POLYGON_NAME));

  nav_msgs::msg::OccupancyGrid::SharedPtr map = makeMap();

  polygon_->putBorders(map, nav2_map_server::OverlayType::OVERLAY_SEQ);

  verifyMapEmpty(map);
}

TEST_F(Tester, testPolygonBordersOnePointInsideBoundaries)
{
  setPolygonParams("");
  node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".closed", false));
  // Now, set the first point inside the map and the others out of the map
  node_->set_parameter(
    rclcpp::Parameter(
      std::string(POLYGON_NAME) + ".points",
      std::vector<double>{0.5, 0.5, 6.0, 5.0, 5.0, 6.0}));
  ASSERT_TRUE(polygon_->obtainParams(POLYGON_NAME));

  nav_msgs::msg::OccupancyGrid::SharedPtr map = makeMap();

  polygon_->putBorders(map, nav2_map_server::OverlayType::OVERLAY_SEQ);

  verifyMapEmpty(map);
}

TEST_F(Tester, testPolygonDifferentFrame)
{
  setPolygonParams("");
  // Change shape's frame ID to differ from global one
  node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".frame_id", SHAPE_FRAME_ID));
  ASSERT_TRUE(polygon_->obtainParams(POLYGON_NAME));

  sendTransform();

  // Check that shape coordinates are not transformed
  geometry_msgs::msg::Polygon::SharedPtr poly = polygon_->getPoly();
  ASSERT_NEAR(poly->points[0].x, 1.0, EPSILON);
  ASSERT_NEAR(poly->points[0].y, 1.0, EPSILON);
  ASSERT_NEAR(poly->points[1].x, -1.0, EPSILON);
  ASSERT_NEAR(poly->points[1].y, 1.0, EPSILON);
  ASSERT_NEAR(poly->points[2].x, -1.0, EPSILON);
  ASSERT_NEAR(poly->points[2].y, -1.0, EPSILON);
  ASSERT_NEAR(poly->points[3].x, 1.0, EPSILON);
  ASSERT_NEAR(poly->points[3].y, -1.0, EPSILON);

  // Transform shape coordinates to global frame
  ASSERT_TRUE(polygon_->toFrame(GLOBAL_FRAME_ID, tf_buffer_, 1.0));

  // Verify that shape coordinates were transformed to global frame successfully
  poly = polygon_->getPoly();
  ASSERT_NEAR(poly->points[0].x, 1.0 + FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(poly->points[0].y, 1.0 + FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(poly->points[1].x, -1.0 + FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(poly->points[1].y, 1.0 + FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(poly->points[2].x, -1.0 + FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(poly->points[2].y, -1.0 + FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(poly->points[3].x, 1.0 + FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(poly->points[3].y, -1.0 + FRAME_SHIFT, EPSILON);

  // Try to transform to incorrect frame
  ASSERT_FALSE(polygon_->toFrame("incorrect_frame", tf_buffer_, 0.1));
}

//---------- Circles testcases ----------

TEST_F(Tester, testCircleObtainParams)
{
  setCircleParams("01010101-0101-0101-0101-010101010101");
  ASSERT_TRUE(circle_->obtainParams(CIRCLE_NAME));

  ASSERT_EQ(circle_->getType(), nav2_map_server::CIRCLE);
  ASSERT_EQ(circle_->getValue(), nav2_util::OCC_GRID_OCCUPIED);
  ASSERT_EQ(circle_->getFrameID(), GLOBAL_FRAME_ID);
  ASSERT_EQ(circle_->isFill(), true);
  ASSERT_EQ(circle_->getUUID(), "01010101-0101-0101-0101-010101010101");
}

TEST_F(Tester, testCircleObtainIncorrectParams)
{
  const std::string circle_name(CIRCLE_NAME);

  // Setting circle parameters w/o center and radius
  node_->declare_parameter(
    circle_name + ".type", rclcpp::ParameterValue("circle"));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".type", "circle"));

  node_->declare_parameter(
    circle_name + ".frame_id", rclcpp::ParameterValue(GLOBAL_FRAME_ID));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".frame_id", GLOBAL_FRAME_ID));

  node_->declare_parameter(
    circle_name + ".fill", rclcpp::ParameterValue(true));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".fill", true));

  node_->declare_parameter(
    circle_name + ".value", rclcpp::ParameterValue(nav2_util::OCC_GRID_OCCUPIED));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".value", nav2_util::OCC_GRID_OCCUPIED));

  node_->declare_parameter(
    circle_name + ".center", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node_->declare_parameter(
    circle_name + ".radius", rclcpp::PARAMETER_DOUBLE);

  // Check that center and radius are mandatory parameter for the circle
  ASSERT_FALSE(circle_->obtainParams(circle_name));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".center", CENTER));

  // Setting incorrect radius
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".radius", -1.0));
  ASSERT_FALSE(circle_->obtainParams(circle_name));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".radius", 1.0));

  // Setting incorrect center format
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".center", std::vector<double>{0.0}));
  ASSERT_FALSE(circle_->obtainParams(circle_name));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".center", CENTER));

  // Setting incorrect UUID
  node_->declare_parameter(
    circle_name + ".uuid", rclcpp::ParameterValue("incorrect-uuid"));
  node_->set_parameter(
    rclcpp::Parameter(circle_name + ".uuid", "incorrect-uuid"));
  ASSERT_FALSE(circle_->obtainParams(circle_name));
}

TEST_F(Tester, testCircleSetParams)
{
  nav2_msgs::msg::CircleObject::SharedPtr co = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2});

  // Check that parameters were set correctly
  ASSERT_TRUE(circle_->setParams(co));
  ASSERT_EQ(circle_->getType(), nav2_map_server::CIRCLE);
  ASSERT_EQ(circle_->getValue(), nav2_util::OCC_GRID_OCCUPIED);
  ASSERT_EQ(circle_->getFrameID(), GLOBAL_FRAME_ID);
  ASSERT_EQ(circle_->isFill(), true);
  ASSERT_EQ(circle_->getUUID(), "01010101-0101-0101-0101-020202020202");

  // Verify that getting the parameters also works correctly
  nav2_msgs::msg::CircleObject::SharedPtr params = circle_->getParams();
  compareCircleObjects(params, co);
}

TEST_F(Tester, testCircleSetIncorrectParams)
{
  nav2_msgs::msg::CircleObject::SharedPtr co = makeCircleObject(
    std::vector<unsigned char>());
  // Setting incorrect radius
  co->radius = -1.0;

  // And check that it is failed to set these parameters
  ASSERT_FALSE(circle_->setParams(co));
}

TEST_F(Tester, testCircleBoundaries)
{
  setCircleParams("");
  ASSERT_TRUE(circle_->obtainParams(CIRCLE_NAME));

  double min_x, min_y, max_x, max_y;
  circle_->getBoundaries(min_x, min_y, max_x, max_y);

  ASSERT_NEAR(min_x, -1.0, EPSILON);
  ASSERT_NEAR(min_y, -1.0, EPSILON);
  ASSERT_NEAR(max_x, 1.0, EPSILON);
  ASSERT_NEAR(max_y, 1.0, EPSILON);
}

TEST_F(Tester, testCirclePoints)
{
  setCircleParams("");
  ASSERT_TRUE(circle_->obtainParams(CIRCLE_NAME));

  ASSERT_TRUE(circle_->isPointInside(0.0, 0.0));
  ASSERT_FALSE(circle_->isPointInside(-1.0, -1.0));
  ASSERT_FALSE(circle_->isPointInside(1.0, 1.0));
}

TEST_F(Tester, testCircleBorders)
{
  setCircleParams("");
  node_->set_parameter(
    rclcpp::Parameter(std::string(CIRCLE_NAME) + ".fill", false));
  ASSERT_TRUE(circle_->obtainParams(CIRCLE_NAME));

  nav_msgs::msg::OccupancyGrid::SharedPtr map = makeMap();

  circle_->putBorders(map, nav2_map_server::OverlayType::OVERLAY_SEQ);

  verifyCircleBorders(map);
}

TEST_F(Tester, testCircleBordersOutOfBoundaries)
{
  setCircleParams("");
  node_->set_parameter(
    rclcpp::Parameter(std::string(CIRCLE_NAME) + ".fill", false));
  // Set circle to be out of map
  node_->set_parameter(
    rclcpp::Parameter(
      std::string(CIRCLE_NAME) + ".center",
      std::vector<double>{5.0, 5.0}));
  ASSERT_TRUE(circle_->obtainParams(CIRCLE_NAME));

  nav_msgs::msg::OccupancyGrid::SharedPtr map = makeMap();

  circle_->putBorders(map, nav2_map_server::OverlayType::OVERLAY_SEQ);

  verifyMapEmpty(map);
}

TEST_F(Tester, testCircleBordersOutsideMap)
{
  setCircleParams("");
  node_->set_parameter(
    rclcpp::Parameter(std::string(CIRCLE_NAME) + ".fill", false));
  // Set circle to be out of map
  node_->set_parameter(
    rclcpp::Parameter(
      std::string(CIRCLE_NAME) + ".center",
      std::vector<double>{-3.0, -3.0}));
  ASSERT_TRUE(circle_->obtainParams(CIRCLE_NAME));

  nav_msgs::msg::OccupancyGrid::SharedPtr map = makeMap();

  circle_->putBorders(map, nav2_map_server::OverlayType::OVERLAY_SEQ);

  verifyMapEmpty(map);
}

TEST_F(Tester, testCircleDifferentFrame)
{
  setCircleParams("");
  // Change shape's frame ID to differ from global one
  node_->set_parameter(
    rclcpp::Parameter(std::string(CIRCLE_NAME) + ".frame_id", SHAPE_FRAME_ID));
  ASSERT_TRUE(circle_->obtainParams(CIRCLE_NAME));

  sendTransform();

  // Check that shape coordinates are not transformed
  geometry_msgs::msg::Point32::SharedPtr center = circle_->getCenter();
  ASSERT_NEAR(center->x, 0.0, EPSILON);
  ASSERT_NEAR(center->y, 0.0, EPSILON);
  // Transform shape coordinates to global frame
  ASSERT_TRUE(circle_->toFrame(GLOBAL_FRAME_ID, tf_buffer_, 1.0));

  // Verify that shape coordinates were transformed to global frame successfully
  center = circle_->getCenter();
  ASSERT_NEAR(center->x, FRAME_SHIFT, EPSILON);
  ASSERT_NEAR(center->y, FRAME_SHIFT, EPSILON);

  // Try to transform to incorrect frame
  ASSERT_FALSE(circle_->toFrame("incorrect_frame", tf_buffer_, 0.1));
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
