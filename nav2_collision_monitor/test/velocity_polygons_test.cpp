// Copyright (c) 2024 Dexory
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

#include <math.h>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/velocity_polygon.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

static const char BASE_FRAME_ID[]{"base_link"};
static const char POLYGON_PUB_TOPIC[]{"polygon_pub"};
static const char POLYGON_NAME[]{"TestVelocityPolygon"};
static const char SUB_POLYGON_FORWARD_NAME[]{"Forward"};
static const char SUB_POLYGON_BACKWARD_NAME[]{"Backward"};
static const char SUB_POLYGON_LEFT_NAME[]{"Left"};
static const char SUB_POLYGON_RIGHT_NAME[]{"Right"};
static const std::vector<double> FORWARD_POLYGON{
  0.5, 0.5, 0.5, -0.5, 0.0, -0.5, 0.0, 0.5};
static const std::vector<double> BACKWARD_POLYGON{
  0.0, 0.5, 0.0, -0.5, -0.5, -0.5, -0.5, 0.5};
static const std::vector<double> LEFT_POLYGON{
  0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, -0.5};
static const std::vector<double> RIGHT_POLYGON{
  0.5, 0.0, 0.5, -0.5, -0.5, -0.5, 0.0, 0.0};
static const char FORWARD_POLYGON_STR[]{
  "[[0.5, 0.5], [0.5, -0.5], [0.0, -0.5], [0.0, 0.5]]"};
static const char BACKWARD_POLYGON_STR[]{
  "[[0.0, 0.5], [0.0, -0.5], [-0.5, -0.5], [-0.5, 0.5]]"};
static const char LEFT_POLYGON_STR[]{
  "[[0.5, 0.5], [0.5, 0.0], [0.0, 0.0], [0.0, -0.5]]"};
static const char RIGHT_POLYGON_STR[]{
  "[[0.5, 0.0], [0.5, -0.5], [-0.5, -0.5], [0.0, 0.0]]"};

static const bool IS_HOLONOMIC{true};
static const bool IS_NOT_HOLONOMIC{false};
static const int MIN_POINTS{2};
static const tf2::Duration TRANSFORM_TOLERANCE{tf2::durationFromSec(0.1)};

class TestNode : public nav2_util::LifecycleNode
{
public:
  TestNode()
  : nav2_util::LifecycleNode("test_node"), polygon_received_(nullptr)
  {
    polygon_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      POLYGON_PUB_TOPIC, rclcpp::SystemDefaultsQoS(),
      std::bind(&TestNode::polygonCallback, this, std::placeholders::_1));
  }

  ~TestNode() {}

  void polygonCallback(geometry_msgs::msg::PolygonStamped::SharedPtr msg)
  {
    polygon_received_ = msg;
  }

  geometry_msgs::msg::PolygonStamped::SharedPtr waitPolygonReceived(
    const std::chrono::nanoseconds & timeout)
  {
    rclcpp::Time start_time = this->now();
    while (rclcpp::ok() && this->now() - start_time <= rclcpp::Duration(timeout)) {
      if (polygon_received_) {
        return polygon_received_;
      }
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(10ms);
    }
    return nullptr;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
  geometry_msgs::msg::PolygonStamped::SharedPtr polygon_received_;
};  // TestNode

class VelocityPolygonWrapper : public nav2_collision_monitor::VelocityPolygon
{
public:
  VelocityPolygonWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance)
  : nav2_collision_monitor::VelocityPolygon(
      node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
  {
  }

  double isHolonomic() const
  {
    return holonomic_;
  }

  double isVisualize() const
  {
    return visualize_;
  }

  std::vector<SubPolygonParameter> getSubPolygons()
  {
    return sub_polygons_;
  }
};  // VelocityPolygonWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

protected:
  // Working with parameters
  void setCommonParameters(const std::string & polygon_name, const std::string & action_type);
  void setVelocityPolygonParameters(const bool is_holonomic);
  void addPolygonVelocitySubPolygon(
    const std::string & sub_polygon_name,
    const double linear_min, const double linear_max,
    const double theta_min, const double theta_max,
    const double direction_end_angle, const double direction_start_angle,
    const std::string & polygon_points, const bool is_holonomic);

  // Creating routines
  void createVelocityPolygon(const std::string & action_type, const bool is_holonomic);

  // Wait until polygon will be received
  bool waitPolygon(
    const std::chrono::nanoseconds & timeout,
    std::vector<nav2_collision_monitor::Point> & poly);

  std::shared_ptr<TestNode> test_node_;

  std::shared_ptr<VelocityPolygonWrapper> velocity_polygon_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};  // Tester

Tester::Tester()
{
  test_node_ = std::make_shared<TestNode>();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Tester::~Tester()
{
  velocity_polygon_.reset();

  test_node_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();
}

void Tester::setCommonParameters(const std::string & polygon_name, const std::string & action_type)
{
  test_node_->declare_parameter(
    polygon_name + ".action_type", rclcpp::ParameterValue(action_type));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".action_type", action_type));

  test_node_->declare_parameter(
    polygon_name + ".min_points", rclcpp::ParameterValue(MIN_POINTS));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".min_points", MIN_POINTS));

  test_node_->declare_parameter(
    polygon_name + ".visualize", rclcpp::ParameterValue(true));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".visualize", true));

  test_node_->declare_parameter(
    polygon_name + ".polygon_pub_topic", rclcpp::ParameterValue(POLYGON_PUB_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(polygon_name + ".polygon_pub_topic", POLYGON_PUB_TOPIC));

  std::vector<std::string> default_observation_sources = {"source"};
  test_node_->declare_parameter(
    "observation_sources", rclcpp::ParameterValue(default_observation_sources));
  test_node_->set_parameter(
    rclcpp::Parameter("observation_sources", default_observation_sources));
}

void Tester::setVelocityPolygonParameters(const bool is_holonomic)
{
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".holonomic", rclcpp::ParameterValue(is_holonomic));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".holonomic", is_holonomic));

  std::vector<std::string> velocity_polygons =
  {SUB_POLYGON_FORWARD_NAME, SUB_POLYGON_BACKWARD_NAME};

  if (is_holonomic) {
    // Direction angle range for holonomic type
    //
    //                    ^OY
    //                    |
    //                    |
    //        0.75pi    (left)    0.25pi
    //             ---------------  <- robot footprint
    //             | \    |    / |
    //  (backward) |   \  |  /   | (forward)
    // --------pi--|------o------|---------->OX
    //             |   /  | \    |
    //             | /    |   \  |
    //             --------------
    //       -0.75pi   (right)    -0.25pi
    //                    |
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_FORWARD_NAME, 0.0, 0.5, -1.0, 1.0, -M_PI_4, M_PI_4, FORWARD_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_BACKWARD_NAME, -0.5, 0.0, -1.0, 1.0, 0.75 * M_PI, -0.75 * M_PI,
      BACKWARD_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_LEFT_NAME, -0.5, 0.5, -1.0, 1.0, M_PI_4, 0.75 * M_PI, LEFT_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_RIGHT_NAME, -0.5, 0.5, -1.0, 1.0, -0.75 * M_PI, -M_PI_4,
      RIGHT_POLYGON_STR, is_holonomic);

    velocity_polygons = {SUB_POLYGON_FORWARD_NAME, SUB_POLYGON_BACKWARD_NAME, SUB_POLYGON_LEFT_NAME,
      SUB_POLYGON_RIGHT_NAME};
  } else {
    // draw forward and backward polygon
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_FORWARD_NAME, 0.0, 0.5, -1.0, 1.0, 0.0, 0.0, FORWARD_POLYGON_STR,
      is_holonomic);
    addPolygonVelocitySubPolygon(
      SUB_POLYGON_BACKWARD_NAME, -0.5, 0.0, -1.0, 1.0, 0.0, 0.0, BACKWARD_POLYGON_STR,
      is_holonomic);
    velocity_polygons = {SUB_POLYGON_FORWARD_NAME, SUB_POLYGON_BACKWARD_NAME};
  }

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".velocity_polygons", rclcpp::ParameterValue(velocity_polygons));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".velocity_polygons", velocity_polygons));
}

void Tester::addPolygonVelocitySubPolygon(
  const std::string & sub_polygon_name,
  const double linear_min, const double linear_max,
  const double theta_min, const double theta_max,
  const double direction_start_angle, const double direction_end_angle,
  const std::string & polygon_points, const bool is_holonomic)
{
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".points",
    rclcpp::ParameterValue(polygon_points));
  test_node_->set_parameter(
    rclcpp::Parameter(
      std::string(
        POLYGON_NAME) +
      "." + sub_polygon_name + ".points",
      polygon_points));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".linear_min",
    rclcpp::ParameterValue(linear_min));
  test_node_->set_parameter(
    rclcpp::Parameter(
      std::string(
        POLYGON_NAME) +
      "." + sub_polygon_name + ".linear_min",
      linear_min));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".linear_max",
    rclcpp::ParameterValue(linear_max));
  test_node_->set_parameter(
    rclcpp::Parameter(
      std::string(
        POLYGON_NAME) +
      "." + sub_polygon_name + ".linear_max",
      linear_max));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".theta_min",
    rclcpp::ParameterValue(theta_min));
  test_node_->set_parameter(
    rclcpp::Parameter(
      std::string(POLYGON_NAME) + "." + sub_polygon_name + ".theta_min",
      theta_min));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + "." + sub_polygon_name + ".theta_max",
    rclcpp::ParameterValue(theta_max));
  test_node_->set_parameter(
    rclcpp::Parameter(
      std::string(POLYGON_NAME) + "." + sub_polygon_name + ".theta_max",
      theta_max));

  if (is_holonomic) {
    test_node_->declare_parameter(
      std::string(
        POLYGON_NAME) +
      "." + sub_polygon_name + ".direction_end_angle",
      rclcpp::ParameterValue(direction_end_angle));
    test_node_->set_parameter(
      rclcpp::Parameter(
        std::string(POLYGON_NAME) + "." + sub_polygon_name + ".direction_end_angle",
        direction_end_angle));

    test_node_->declare_parameter(
      std::string(
        POLYGON_NAME) +
      "." + sub_polygon_name + ".direction_start_angle",
      rclcpp::ParameterValue(direction_start_angle));
    test_node_->set_parameter(
      rclcpp::Parameter(
        std::string(POLYGON_NAME) + "." + sub_polygon_name +
        ".direction_start_angle",
        direction_start_angle));
  }
}

void Tester::createVelocityPolygon(const std::string & action_type, const bool is_holonomic)
{
  setCommonParameters(POLYGON_NAME, action_type);
  setVelocityPolygonParameters(is_holonomic);

  velocity_polygon_ = std::make_shared<VelocityPolygonWrapper>(
    test_node_, POLYGON_NAME,
    tf_buffer_, BASE_FRAME_ID, TRANSFORM_TOLERANCE);
  ASSERT_TRUE(velocity_polygon_->configure());
  velocity_polygon_->activate();
}

bool Tester::waitPolygon(
  const std::chrono::nanoseconds & timeout,
  std::vector<nav2_collision_monitor::Point> & poly)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    velocity_polygon_->getPolygon(poly);
    if (poly.size() > 0) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

TEST_F(Tester, testVelocityPolygonGetStopParameters)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check that common parameters set correctly
  EXPECT_EQ(velocity_polygon_->getName(), POLYGON_NAME);
  EXPECT_EQ(velocity_polygon_->getActionType(), nav2_collision_monitor::STOP);
  EXPECT_EQ(velocity_polygon_->getMinPoints(), MIN_POINTS);
  EXPECT_EQ(velocity_polygon_->isVisualize(), true);
}

TEST_F(Tester, testVelocityPolygonGetSlowdownParameters)
{
  createVelocityPolygon("slowdown", IS_NOT_HOLONOMIC);

  // Check that common parameters set correctly
  EXPECT_EQ(velocity_polygon_->getName(), POLYGON_NAME);
  EXPECT_EQ(velocity_polygon_->getActionType(), nav2_collision_monitor::SLOWDOWN);
  EXPECT_EQ(velocity_polygon_->getMinPoints(), MIN_POINTS);
  EXPECT_EQ(velocity_polygon_->isVisualize(), true);
}

TEST_F(Tester, testVelocityPolygonParameters)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_NOT_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 2u);
}

TEST_F(Tester, testHolonomicVelocityPolygonParameters)
{
  createVelocityPolygon("stop", IS_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 4u);
}

TEST_F(Tester, testVelocityPolygonOutOfRangeVelocity)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_NOT_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 2u);

  // Check that polygon is empty before the first cmd_vel received
  std::vector<nav2_collision_monitor::Point> poly;
  velocity_polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 0u);


  // Publish out of range cmd_vel(linear) and check that polygon is still emtpy
  nav2_collision_monitor::Velocity vel{0.6, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_FALSE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 0u);

  // Publish out of range cmd_vel(rotation) and check that polygon is still emtpy
  vel = {0.3, 0.0, 1.5};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_FALSE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 0u);

  // Publish a valid cmd_vel and check that polygon is correct
  vel = {0.3, 0.0, 0.0};  // 0.3 m/s forward movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
}

TEST_F(Tester, testVelocityPolygonVelocitySwitching)
{
  createVelocityPolygon("stop", IS_NOT_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_NOT_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 2u);

  // Check that polygon is empty before the first cmd_vel received
  std::vector<nav2_collision_monitor::Point> poly;
  velocity_polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 0u);

  // Publish cmd_vel (forward) and check that polygon is correct
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, FORWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, FORWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, FORWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, FORWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, FORWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, FORWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, FORWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, FORWARD_POLYGON[7], EPSILON);

  // Publish cmd_vel (backward) and check that polygon is correct
  vel = {-0.3, 0.0, 0.0};  // 0.3 m/s backward movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, BACKWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, BACKWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, BACKWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, BACKWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, BACKWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, BACKWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, BACKWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, BACKWARD_POLYGON[7], EPSILON);
}

TEST_F(Tester, testVelocityPolygonHolonomicVelocitySwitching)
{
  createVelocityPolygon("stop", IS_HOLONOMIC);

  // Check velocity polygon parameters
  EXPECT_EQ(velocity_polygon_->isHolonomic(), IS_HOLONOMIC);
  ASSERT_EQ(velocity_polygon_->getSubPolygons().size(), 4u);

  // Check that polygon is empty before the first cmd_vel received
  std::vector<nav2_collision_monitor::Point> poly;
  velocity_polygon_->getPolygon(poly);
  ASSERT_EQ(poly.size(), 0u);

  // Publish cmd_vel (forward) and check that polygon is correct
  nav2_collision_monitor::Velocity vel{0.3, 0.0, 0.0};
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, FORWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, FORWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, FORWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, FORWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, FORWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, FORWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, FORWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, FORWARD_POLYGON[7], EPSILON);

  // Publish cmd_vel (backward) and check that polygon is correct
  vel = {-0.3, 0.0, 0.0};  // 0.3 m/s backward movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, BACKWARD_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, BACKWARD_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, BACKWARD_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, BACKWARD_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, BACKWARD_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, BACKWARD_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, BACKWARD_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, BACKWARD_POLYGON[7], EPSILON);

  // Publish cmd_vel (left) and check that polygon is correct
  vel = {0.0, 0.3, 0.0};  // 0.3 m/s left movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, LEFT_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, LEFT_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, LEFT_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, LEFT_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, LEFT_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, LEFT_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, LEFT_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, LEFT_POLYGON[7], EPSILON);

  // Publish cmd_vel (right) and check that polygon is correct
  vel = {0.0, -0.3, 0.0};  // 0.3 m/s right movement
  velocity_polygon_->updatePolygon(vel);
  ASSERT_TRUE(waitPolygon(500ms, poly));
  ASSERT_EQ(poly.size(), 4u);
  EXPECT_NEAR(poly[0].x, RIGHT_POLYGON[0], EPSILON);
  EXPECT_NEAR(poly[0].y, RIGHT_POLYGON[1], EPSILON);
  EXPECT_NEAR(poly[1].x, RIGHT_POLYGON[2], EPSILON);
  EXPECT_NEAR(poly[1].y, RIGHT_POLYGON[3], EPSILON);
  EXPECT_NEAR(poly[2].x, RIGHT_POLYGON[4], EPSILON);
  EXPECT_NEAR(poly[2].y, RIGHT_POLYGON[5], EPSILON);
  EXPECT_NEAR(poly[3].x, RIGHT_POLYGON[6], EPSILON);
  EXPECT_NEAR(poly[3].y, RIGHT_POLYGON[7], EPSILON);
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
