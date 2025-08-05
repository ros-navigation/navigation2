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
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav2_msgs/srv/add_shapes.hpp"
#include "nav2_msgs/srv/get_shapes.hpp"
#include "nav2_msgs/srv/remove_shapes.hpp"
#include "nav2_msgs/msg/polygon_object.hpp"
#include "nav2_msgs/msg/circle_object.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/occ_grid_utils.hpp"

#include "nav2_map_server/vector_object_server.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

static const char POLYGON_NAME[]{"Polygon"};
static const char CIRCLE_NAME[]{"Circle"};
static const char GLOBAL_FRAME_ID[]{"map"};
static const char SHAPE_FRAME_ID[]{"shape"};
static const double UPDATE_FREQUENCY{10.0};
static const int8_t POLYGON_VAL{nav2_util::OCC_GRID_OCCUPIED};
static const int8_t CIRCLE_VAL{nav2_util::OCC_GRID_OCCUPIED / 2};

class VOServerWrapper : public nav2_map_server::VectorObjectServer
{
public:
  void start()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_activate(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  void configureFail()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2::CallbackReturn::FAILURE);
  }

  void stop()
  {
    ASSERT_EQ(on_deactivate(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_cleanup(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_shutdown(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  void cleanup()
  {
    ASSERT_EQ(on_cleanup(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_shutdown(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  void setProcessMap(const bool process_map)
  {
    process_map_ = process_map;
  }

  void getMapBoundaries(double & min_x, double & min_y, double & max_x, double & max_y) const
  {
    VectorObjectServer::getMapBoundaries(min_x, min_y, max_x, max_y);
  }

  void updateMap(
    const double & min_x, const double & min_y, const double & max_x, const double & max_y)
  {
    VectorObjectServer::updateMap(min_x, min_y, max_x, max_y);
  }

  void putVectorObjectsOnMap()
  {
    VectorObjectServer::putVectorObjectsOnMap();
  }
};  // VOServerWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

  void setVOServerParams();
  void setShapesParams(const std::string & poly_uuid, const std::string & circle_uuid);
  void setPolygonParams(const std::string & uuid);
  void setCircleParams(const std::string & uuid);

  nav2_msgs::msg::PolygonObject::SharedPtr makePolygonObject(
    const std::vector<unsigned char> & uuid);
  nav2_msgs::msg::CircleObject::SharedPtr makeCircleObject(
    const std::vector<unsigned char> & uuid);

  void sendTransform(const double frame_shift);

  template<class T>
  typename T::Response::SharedPtr sendRequest(
    typename nav2::ServiceClient<T>::SharedPtr client,
    typename T::Request::SharedPtr request,
    const std::chrono::nanoseconds & timeout);

  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map);
  bool waitMap(const std::chrono::nanoseconds & timeout);
  void verifyMap(bool is_circle, double poly_x_end = 1.0, double circle_cx = 3.0);

  void comparePolygonObjects(
    nav2_msgs::msg::PolygonObject::SharedPtr p1,
    nav2_msgs::msg::PolygonObject::SharedPtr p2);
  void compareCircleObjects(
    nav2_msgs::msg::CircleObject::SharedPtr c1,
    nav2_msgs::msg::CircleObject::SharedPtr c2);

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Service clients for calling AddShapes.srv, GetShapes.srv, RemoveShapes.srv
  nav2::ServiceClient<nav2_msgs::srv::AddShapes>::SharedPtr add_shapes_client_;
  nav2::ServiceClient<nav2_msgs::srv::GetShapes>::SharedPtr get_shapes_client_;
  nav2::ServiceClient<nav2_msgs::srv::RemoveShapes>::SharedPtr remove_shapes_client_;

  // Output map subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr vo_map_sub_;
  // Output map published by VectorObjectServer
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  // Vector Object server node
  std::shared_ptr<VOServerWrapper> vo_server_;
};  // Tester

Tester::Tester()
: map_(nullptr)
{
  vo_server_ = std::make_shared<VOServerWrapper>();

  add_shapes_client_ = vo_server_->create_client<nav2_msgs::srv::AddShapes>(
    std::string(vo_server_->get_name()) + "/add_shapes");

  get_shapes_client_ = vo_server_->create_client<nav2_msgs::srv::GetShapes>(
    std::string(vo_server_->get_name()) + "/get_shapes");

  remove_shapes_client_ = vo_server_->create_client<nav2_msgs::srv::RemoveShapes>(
    std::string(vo_server_->get_name()) + "/remove_shapes");

  vo_map_sub_ = vo_server_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "vo_map", std::bind(&Tester::mapCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS());

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(vo_server_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Tester::~Tester()
{
  vo_map_sub_.reset();

  add_shapes_client_.reset();
  get_shapes_client_.reset();
  remove_shapes_client_.reset();

  vo_server_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();
}

void Tester::setVOServerParams()
{
  vo_server_->declare_parameter(
    "map_topic", rclcpp::ParameterValue("vo_map"));
  vo_server_->set_parameter(
    rclcpp::Parameter("map_topic", "vo_map"));

  vo_server_->declare_parameter(
    "global_frame_id", rclcpp::ParameterValue("map"));
  vo_server_->set_parameter(
    rclcpp::Parameter("global_frame_id", "map"));

  vo_server_->declare_parameter(
    "resolution", rclcpp::ParameterValue(0.1));
  vo_server_->set_parameter(
    rclcpp::Parameter("resolution", 0.1));

  vo_server_->declare_parameter(
    "default_value", rclcpp::ParameterValue(nav2_util::OCC_GRID_UNKNOWN));
  vo_server_->set_parameter(
    rclcpp::Parameter("default_value", nav2_util::OCC_GRID_UNKNOWN));

  vo_server_->declare_parameter(
    "overlay_type",
    rclcpp::ParameterValue(static_cast<int>(nav2_map_server::OverlayType::OVERLAY_SEQ)));
  vo_server_->set_parameter(
    rclcpp::Parameter(
      "overlay_type",
      static_cast<int>(nav2_map_server::OverlayType::OVERLAY_SEQ)));

  vo_server_->declare_parameter(
    "update_frequency", rclcpp::ParameterValue(UPDATE_FREQUENCY));
  vo_server_->set_parameter(
    rclcpp::Parameter("update_frequency", UPDATE_FREQUENCY));

  vo_server_->declare_parameter(
    "transform_tolerance", rclcpp::ParameterValue(0.1));
  vo_server_->set_parameter(
    rclcpp::Parameter("transform_tolerance", 0.1));
}

void Tester::setShapesParams(const std::string & poly_uuid, const std::string & circle_uuid)
{
  vo_server_->declare_parameter(
    "shapes", rclcpp::ParameterValue(std::vector<std::string>{POLYGON_NAME, CIRCLE_NAME}));
  vo_server_->set_parameter(
    rclcpp::Parameter("shapes", std::vector<std::string>{POLYGON_NAME, CIRCLE_NAME}));

  setPolygonParams(poly_uuid);
  setCircleParams(circle_uuid);
}

void Tester::setPolygonParams(const std::string & uuid)
{
  const std::string polygon_name(POLYGON_NAME);

  vo_server_->declare_parameter(
    polygon_name + ".type", rclcpp::ParameterValue("polygon"));
  vo_server_->set_parameter(
    rclcpp::Parameter(polygon_name + ".type", "polygon"));

  vo_server_->declare_parameter(
    polygon_name + ".frame_id", rclcpp::ParameterValue(GLOBAL_FRAME_ID));
  vo_server_->set_parameter(
    rclcpp::Parameter(polygon_name + ".frame_id", GLOBAL_FRAME_ID));

  vo_server_->declare_parameter(
    polygon_name + ".closed", rclcpp::ParameterValue(true));
  vo_server_->set_parameter(
    rclcpp::Parameter(polygon_name + ".closed", true));

  vo_server_->declare_parameter(
    polygon_name + ".value", rclcpp::ParameterValue(POLYGON_VAL));
  vo_server_->set_parameter(
    rclcpp::Parameter(polygon_name + ".value", POLYGON_VAL));

  std::vector<double> points{1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0};
  vo_server_->declare_parameter(
    polygon_name + ".points", rclcpp::ParameterValue(points));
  vo_server_->set_parameter(
    rclcpp::Parameter(polygon_name + ".points", points));

  if (!uuid.empty()) {
    vo_server_->declare_parameter(
      polygon_name + ".uuid", rclcpp::ParameterValue(uuid));
    vo_server_->set_parameter(
      rclcpp::Parameter(polygon_name + ".uuid", uuid));
  }
}

void Tester::setCircleParams(const std::string & uuid)
{
  const std::string circle_name(CIRCLE_NAME);

  vo_server_->declare_parameter(
    circle_name + ".type", rclcpp::ParameterValue("circle"));
  vo_server_->set_parameter(
    rclcpp::Parameter(circle_name + ".type", "circle"));

  vo_server_->declare_parameter(
    circle_name + ".frame_id", rclcpp::ParameterValue(GLOBAL_FRAME_ID));
  vo_server_->set_parameter(
    rclcpp::Parameter(circle_name + ".frame_id", GLOBAL_FRAME_ID));

  vo_server_->declare_parameter(
    circle_name + ".fill", rclcpp::ParameterValue(true));
  vo_server_->set_parameter(
    rclcpp::Parameter(circle_name + ".fill", true));

  vo_server_->declare_parameter(
    circle_name + ".value", rclcpp::ParameterValue(CIRCLE_VAL));
  vo_server_->set_parameter(
    rclcpp::Parameter(circle_name + ".value", CIRCLE_VAL));

  vo_server_->declare_parameter(
    circle_name + ".center", rclcpp::ParameterValue(std::vector<double>{3.0, 0.0}));
  vo_server_->set_parameter(
    rclcpp::Parameter(circle_name + ".center", std::vector<double>{3.0, 0.0}));

  vo_server_->declare_parameter(
    circle_name + ".radius", rclcpp::ParameterValue(1.0));
  vo_server_->set_parameter(
    rclcpp::Parameter(circle_name + ".radius", 1.0));

  if (!uuid.empty()) {
    vo_server_->declare_parameter(
      circle_name + ".uuid", rclcpp::ParameterValue(uuid));
    vo_server_->set_parameter(
      rclcpp::Parameter(circle_name + ".uuid", uuid));
  }
}

void Tester::sendTransform(const double frame_shift)
{
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(vo_server_);

  geometry_msgs::msg::TransformStamped transform;

  // global_frame -> shape_frame transform
  transform.header.frame_id = GLOBAL_FRAME_ID;
  transform.child_frame_id = SHAPE_FRAME_ID;

  transform.header.stamp = vo_server_->now();
  transform.transform.translation.x = frame_shift;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster->sendTransform(transform);
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
  po->value = POLYGON_VAL;

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
  co->center.x = 3.0;
  co->center.y = 0.0;
  co->radius = 1.0;
  co->fill = true;
  co->value = CIRCLE_VAL;

  return co;
}

template<class T>
typename T::Response::SharedPtr Tester::sendRequest(
  typename nav2::ServiceClient<T>::SharedPtr client,
  typename T::Request::SharedPtr request,
  const std::chrono::nanoseconds & timeout)
{
  auto result_future = client->async_call(request);

  rclcpp::Time start_time = vo_server_->now();
  while (rclcpp::ok() && vo_server_->now() - start_time <= rclcpp::Duration(timeout)) {
    std::future_status status = result_future.wait_for(10ms);
    if (status == std::future_status::ready) {
      return result_future.get();
    }
    rclcpp::spin_some(vo_server_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return nullptr;
}

void Tester::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  map_ = map;
}

bool Tester::waitMap(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = vo_server_->now();
  while (rclcpp::ok() && vo_server_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (map_) {
      return true;
    }
    rclcpp::spin_some(vo_server_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void Tester::verifyMap(bool is_circle, double poly_x_end, double circle_cx)
{
  // Wait for map to be received
  const std::chrono::nanoseconds timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(1s * 1.5 / UPDATE_FREQUENCY);
  ASSERT_TRUE(waitMap(timeout));

  // Map should contain polygon and circle and will look like:
  //
  // polygon {x1, y1} (and map's origin): should be always {-1.0, -1.0}
  // V
  // xxxxxx....xxx.
  // xxxxxx...xxxxx < circle is optionally placed on map
  // xxxxxx...xxxxx
  // xxxxxx....xxx.
  //      ^
  //      polygon {x2, y2}. x2 = poly_x_end; y2 - is always == 1.0

  // Polygon {x2, y2} in map coordinates
  const unsigned int m_poly_x2 = 9 + poly_x_end * 10;
  const unsigned int m_poly_y2 = 19;

  // Center of the circle in map coordinates (expressed in floating-point for best precision)
  const double m_circle_cx = 9 + circle_cx * 10 + 0.5;  // cell's origin + 0.5 center of cell
  const double m_circle_cy = 9 + 0.5;  // cell's origin + 0.5 center of cell

  // Radius of the circle in map coordinates
  const double m_circle_rad = 10.0;

  for (unsigned int my = 0; my < map_->info.height; my++) {
    for (unsigned int mx = 0; mx < map_->info.width; mx++) {
      if (mx <= m_poly_x2 && my <= m_poly_y2) {
        // Point belongs to the polygon
        ASSERT_EQ(map_->data[my * map_->info.width + mx], POLYGON_VAL);
      } else if (is_circle && std::hypot(m_circle_cx - mx, m_circle_cy - my) <= m_circle_rad) {
        // Point belongs to the circle
        ASSERT_EQ(map_->data[my * map_->info.width + mx], CIRCLE_VAL);
      } else {
        // Point does not belong to any shape
        ASSERT_EQ(map_->data[my * map_->info.width + mx], nav2_util::OCC_GRID_UNKNOWN);
      }
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

// ---------- ROS-parameters tests ----------
TEST_F(Tester, testObtainParams)
{
  setVOServerParams();
  setShapesParams(
    "01010101-0101-0101-0101-010101010101",
    "01010101-0101-0101-0101-010101010102");
  vo_server_->start();

  verifyMap(true);

  vo_server_->stop();
}

TEST_F(Tester, testObtainNoShapes)
{
  setVOServerParams();
  // Set shapes array, but does not set shapes themselves
  vo_server_->declare_parameter(
    "shapes", rclcpp::ParameterValue(std::vector<std::string>{POLYGON_NAME, CIRCLE_NAME}));
  vo_server_->set_parameter(
    rclcpp::Parameter("shapes", std::vector<std::string>{POLYGON_NAME, CIRCLE_NAME}));

  // Configured that way, vo_server_ should give the failure on configuration stage
  vo_server_->configureFail();

  // Cleaning-up vo_server_ w/o deactivation
  vo_server_->cleanup();
}

TEST_F(Tester, testIncorrectShapeType)
{
  setVOServerParams();
  setShapesParams("", "");
  // Setting incorrect type of circle
  vo_server_->set_parameter(
    rclcpp::Parameter(std::string(CIRCLE_NAME) + ".type", "incorrect_type"));

  // Configured that way, vo_server_ should give the failure on configuration stage
  vo_server_->configureFail();

  // Cleaning-up vo_server_ w/o deactivation
  vo_server_->cleanup();
}

TEST_F(Tester, testIncorrectPolygonParams)
{
  setVOServerParams();
  // Setting incorrect UUID. Other incorrect params to be checked inside Vector Object Shapes test
  setShapesParams(
    "incorrect_polygon_uuid",
    "01010101-0101-0101-0101-010101010102");

  // Set that way, vo_server_ should give the failure on configuration stage
  vo_server_->configureFail();

  // Cleaning-up vo_server_ w/o deactivation
  vo_server_->cleanup();
}

TEST_F(Tester, testIncorrectCircleParams)
{
  setVOServerParams();
  // Setting incorrect UUID. Other incorrect params to be checked inside Vector Object Shapes test
  setShapesParams(
    "01010101-0101-0101-0101-010101010101",
    "incorrect_circle_uuid");

  // Set that way, vo_server_ should give the failure on configuration stage
  vo_server_->configureFail();

  // Cleaning-up vo_server_ w/o deactivation
  vo_server_->cleanup();
}

// ---------- Service call tests ----------
TEST_F(Tester, testAddShapes)
{
  setVOServerParams();
  vo_server_->start();

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check the map has been updated correctly
  verifyMap(true);

  // Check that getShapes.srv will return correct shapes
  auto get_shapes_msg = std::make_shared<nav2_msgs::srv::GetShapes::Request>();
  auto get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  auto p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circle
  ASSERT_EQ(get_shapes_result->circles.size(), 1u);
  auto c_check = std::make_shared<nav2_msgs::msg::CircleObject>(get_shapes_result->circles[0]);
  compareCircleObjects(c_check, co_msg);

  // Now move X-coordinate of polygon's border and
  // add a new circle fully placed inside first one
  // through AddShapes.srv:
  // Update polygon x2-coordinate to 1.5
  po_msg->points[0].x = 1.5;
  po_msg->points[3].x = 1.5;
  // Prepare second circle object (fully placed inside first circle)
  auto co2_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3});
  co2_msg->radius = 0.5;

  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.clear();
  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  add_shapes_msg->circles.push_back(*co2_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check the map has been updated correctly
  verifyMap(true, 1.5);

  // Check that getShapes.srv will return correct shapes
  get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circles
  ASSERT_EQ(get_shapes_result->circles.size(), 2u);
  c_check = std::make_shared<nav2_msgs::msg::CircleObject>(get_shapes_result->circles[0]);
  compareCircleObjects(c_check, co_msg);
  c_check = std::make_shared<nav2_msgs::msg::CircleObject>(get_shapes_result->circles[1]);
  compareCircleObjects(c_check, co2_msg);

  vo_server_->stop();
}

TEST_F(Tester, testRemoveShapes)
{
  setVOServerParams();
  vo_server_->start();

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check that getShapes.srv will return correct shapes
  auto get_shapes_msg = std::make_shared<nav2_msgs::srv::GetShapes::Request>();
  auto get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  auto p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circle
  ASSERT_EQ(get_shapes_result->circles.size(), 1u);
  auto c_check = std::make_shared<nav2_msgs::msg::CircleObject>(get_shapes_result->circles[0]);
  compareCircleObjects(c_check, co_msg);

  // Now remove circle from map
  auto remove_shapes_msg = std::make_shared<nav2_msgs::srv::RemoveShapes::Request>();
  remove_shapes_msg->all_objects = false;
  unique_identifier_msgs::msg::UUID uuid;
  uuid.uuid = std::array<unsigned char, 16>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2};
  remove_shapes_msg->uuids.push_back(uuid);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto remove_shapes_result =
    sendRequest<nav2_msgs::srv::RemoveShapes>(remove_shapes_client_, remove_shapes_msg, 2s);
  ASSERT_NE(remove_shapes_result, nullptr);
  ASSERT_TRUE(remove_shapes_result->success);

  // Check that there is only polygon remained on map
  verifyMap(false);

  // Check that getShapes.srv will return only polygon
  get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circle
  ASSERT_EQ(get_shapes_result->circles.size(), 0u);

  // Then call RemoveShapes.srv with enabled all_objects parameter
  remove_shapes_msg->all_objects = true;
  remove_shapes_msg->uuids.clear();
  map_.reset();  // Resetting the map to ensure that updated one is received later
  remove_shapes_result =
    sendRequest<nav2_msgs::srv::RemoveShapes>(remove_shapes_client_, remove_shapes_msg, 2s);
  ASSERT_NE(remove_shapes_result, nullptr);
  ASSERT_TRUE(remove_shapes_result->success);

  // Check that map is empty
  ASSERT_EQ(map_->info.width, 1);
  ASSERT_EQ(map_->info.height, 1);
  ASSERT_EQ(map_->data[0], nav2_util::OCC_GRID_UNKNOWN);

  // getShapes.srv should return empty vectors
  get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  ASSERT_EQ(get_shapes_result->polygons.size(), 0u);
  ASSERT_EQ(get_shapes_result->circles.size(), 0u);

  vo_server_->stop();
}

TEST_F(Tester, testAddIncorrectShapes)
{
  setVOServerParams();
  vo_server_->start();

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check that getShapes.srv will return correct shapes
  auto get_shapes_msg = std::make_shared<nav2_msgs::srv::GetShapes::Request>();
  auto get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  auto p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circle
  ASSERT_EQ(get_shapes_result->circles.size(), 1u);
  auto c_check = std::make_shared<nav2_msgs::msg::CircleObject>(get_shapes_result->circles[0]);
  compareCircleObjects(c_check, co_msg);

  // Then try to update polygon with circle's uuid
  po_msg->uuid.uuid[15] = 2;
  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.clear();
  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_FALSE(add_shapes_result->success);
  po_msg->uuid.uuid[15] = 1;  // Restoring back for further usage

  // Then try to update circle with polygon's uuid
  co_msg->uuid.uuid[15] = 1;
  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.clear();
  add_shapes_msg->circles.push_back(*co_msg);
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_FALSE(add_shapes_result->success);
  co_msg->uuid.uuid[15] = 2;  // Restoring back for further usage

  // Try to update polygon with incorrect number of points
  po_msg->points.resize(2);
  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.clear();
  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_FALSE(add_shapes_result->success);

  // Try to add new incorrect polygon
  po_msg->uuid.uuid[15] = 100;
  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.clear();
  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_FALSE(add_shapes_result->success);
  // Restoring back for further usage
  po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});

  // Try to update circle with incorrect radius
  co_msg->radius = -1.0;
  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.clear();
  add_shapes_msg->circles.push_back(*co_msg);
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_FALSE(add_shapes_result->success);

  // Try to add new incorrect circle
  co_msg->uuid.uuid[15] = 100;
  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.clear();
  add_shapes_msg->circles.push_back(*co_msg);
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_FALSE(add_shapes_result->success);
  co_msg->radius = 1.0;  // Restoring back for further usage
  co_msg->uuid.uuid[15] = 2;  // Restoring back for further usage

  // Verify that map did not corrupted after all false-manipulations
  verifyMap(true);

  // Check that getShapes.srv will return correct shapes after all false-manipulations
  get_shapes_msg = std::make_shared<nav2_msgs::srv::GetShapes::Request>();
  get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circle
  ASSERT_EQ(get_shapes_result->circles.size(), 1u);
  c_check = std::make_shared<nav2_msgs::msg::CircleObject>(get_shapes_result->circles[0]);
  compareCircleObjects(c_check, co_msg);

  vo_server_->stop();
}

TEST_F(Tester, testRemoveIncorrectShapes)
{
  setVOServerParams();
  vo_server_->start();

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check that getShapes.srv will return correct shapes
  auto get_shapes_msg = std::make_shared<nav2_msgs::srv::GetShapes::Request>();
  auto get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  auto p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circle
  ASSERT_EQ(get_shapes_result->circles.size(), 1u);
  auto c_check = std::make_shared<nav2_msgs::msg::CircleObject>(get_shapes_result->circles[0]);
  compareCircleObjects(c_check, co_msg);

  // Then try to remove two shapes: non-existing shape and circle
  auto remove_shapes_msg = std::make_shared<nav2_msgs::srv::RemoveShapes::Request>();
  remove_shapes_msg->all_objects = false;
  unique_identifier_msgs::msg::UUID uuid;
  uuid.uuid = std::array<unsigned char, 16>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 100};
  remove_shapes_msg->uuids.push_back(uuid);
  uuid.uuid = std::array<unsigned char, 16>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2};
  remove_shapes_msg->uuids.push_back(uuid);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto remove_shapes_result =
    sendRequest<nav2_msgs::srv::RemoveShapes>(remove_shapes_client_, remove_shapes_msg, 2s);
  ASSERT_NE(remove_shapes_result, nullptr);
  ASSERT_FALSE(remove_shapes_result->success);

  // Check that despite on the error, the circle was removed from map
  verifyMap(false);

  // Check that getShapes.srv will return only polygon
  get_shapes_result =
    sendRequest<nav2_msgs::srv::GetShapes>(get_shapes_client_, get_shapes_msg, 2s);
  ASSERT_NE(get_shapes_result, nullptr);
  // Verify obtained polygon
  ASSERT_EQ(get_shapes_result->polygons.size(), 1u);
  p_check = std::make_shared<nav2_msgs::msg::PolygonObject>(get_shapes_result->polygons[0]);
  comparePolygonObjects(p_check, po_msg);
  // Verify obtained circle
  ASSERT_EQ(get_shapes_result->circles.size(), 0u);

  vo_server_->stop();
}

// ---------- Overlay tests ----------
TEST_F(Tester, testOverlayMax)
{
  setVOServerParams();
  vo_server_->set_parameter(
    rclcpp::Parameter(
      "overlay_type",
      static_cast<int>(nav2_map_server::OverlayType::OVERLAY_MAX)));
  vo_server_->start();

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});
  // Make circle to be overlapped with polygon
  co_msg->center.x = 1.0;

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Wait for the map
  waitMap(500ms);

  // Verify that overlap on map generated correctly
  double my = 9;
  double mx = 1;  // inside polygon
  ASSERT_EQ(map_->data[my * map_->info.width + mx], POLYGON_VAL);
  mx = 14;  // in the overlapping area
  ASSERT_EQ(map_->data[my * map_->info.width + mx], POLYGON_VAL);
  mx = 20;  // inside circle
  ASSERT_EQ(map_->data[my * map_->info.width + mx], CIRCLE_VAL);
  my = 18;  // outside any shape
  mx = 28;  // outside any shape
  ASSERT_EQ(map_->data[my * map_->info.width + mx], nav2_util::OCC_GRID_UNKNOWN);

  vo_server_->stop();
}

TEST_F(Tester, testOverlayMin)
{
  setVOServerParams();
  vo_server_->set_parameter(
    rclcpp::Parameter(
      "overlay_type",
      static_cast<int>(nav2_map_server::OverlayType::OVERLAY_MIN)));
  vo_server_->start();

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});
  // Make circle to be overlapped with polygon
  co_msg->center.x = 1.0;

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Wait for the map
  waitMap(500ms);

  // Verify that overlap on map generated correctly
  double my = 9;
  double mx = 1;  // inside polygon
  ASSERT_EQ(map_->data[my * map_->info.width + mx], POLYGON_VAL);
  mx = 14;  // in the overlapping area
  ASSERT_EQ(map_->data[my * map_->info.width + mx], CIRCLE_VAL);
  mx = 20;  // inside circle
  ASSERT_EQ(map_->data[my * map_->info.width + mx], CIRCLE_VAL);
  my = 18;  // outside any shape
  mx = 28;  // outside any shape
  ASSERT_EQ(map_->data[my * map_->info.width + mx], nav2_util::OCC_GRID_UNKNOWN);

  vo_server_->stop();
}

TEST_F(Tester, testOverlaySeq)
{
  setVOServerParams();
  vo_server_->set_parameter(
    rclcpp::Parameter(
      "overlay_type",
      static_cast<int>(nav2_map_server::OverlayType::OVERLAY_SEQ)));
  vo_server_->start();

  // Sequentially add polygon and then circle overlapped with it on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();

  // Prepare first polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  add_shapes_msg->polygons.push_back(*po_msg);
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Then prepare circle object to add over the polygon
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});
  // Make circle to be overlapped with polygon
  co_msg->center.x = 1.0;
  // Also check that putBorders part works correctly
  co_msg->fill = false;
  add_shapes_msg->polygons.clear();
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Wait for the map
  waitMap(500ms);

  // Verify that overlap on map generated correctly
  double my = 9;
  double mx = 1;  // inside polygon
  ASSERT_EQ(map_->data[my * map_->info.width + mx], POLYGON_VAL);
  mx = 10;  // on the circle border laying over the polygon
  ASSERT_EQ(map_->data[my * map_->info.width + mx], CIRCLE_VAL);
  mx = 14;  // inside circle and polygon
  ASSERT_EQ(map_->data[my * map_->info.width + mx], POLYGON_VAL);
  mx = 24;  // inside circle only
  ASSERT_EQ(map_->data[my * map_->info.width + mx], nav2_util::OCC_GRID_UNKNOWN);
  mx = 29;  // on the circle border laying outside the polygon
  ASSERT_EQ(map_->data[my * map_->info.width + mx], CIRCLE_VAL);
  my = 18;  // outside any shape
  mx = 28;  // outside any shape
  ASSERT_EQ(map_->data[my * map_->info.width + mx], nav2_util::OCC_GRID_UNKNOWN);

  vo_server_->stop();
}

TEST_F(Tester, testOverlayUnknown)
{
  setVOServerParams();
  vo_server_->set_parameter(
    rclcpp::Parameter("overlay_type", static_cast<int>(1000)));
  vo_server_->start();

  // Try to add polygon on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  auto po_msg = makePolygonObject(std::vector<unsigned char>());
  add_shapes_msg->polygons.push_back(*po_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Wait for the map
  waitMap(500ms);

  // Check that polygon was not added on map: map is empty
  ASSERT_EQ(map_->info.width, 1);
  ASSERT_EQ(map_->info.height, 1);
  ASSERT_EQ(map_->data[0], nav2_util::OCC_GRID_UNKNOWN);

  vo_server_->stop();
}

// ---------- Transform/dynamic tests ----------
TEST_F(Tester, testShapesMove)
{
  setVOServerParams();
  vo_server_->start();

  // Wait for the initial map to be received to not get it in later updates
  const std::chrono::nanoseconds timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(1s * 1.5 / UPDATE_FREQUENCY);
  ASSERT_TRUE(waitMap(timeout));

  // No shift between polygon and map
  sendTransform(0.0);

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Polygon and circle to be in another moving frame
  po_msg->header.frame_id = SHAPE_FRAME_ID;
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check that polygon and circle are in correct positions on map
  verifyMap(true);

  // Move frame with polygon
  sendTransform(0.5);

  // Map is being published dynamically. Wait for the map to be published one more time
  map_.reset();
  // Check that polygon and circle are in correct positions on map
  verifyMap(true, 1.0, 2.5);  // Polygon is moved 0.5m towards to the circle
  // Check that map's origin was updated in accordance to polygon movement
  ASSERT_NEAR(map_->info.origin.position.x, -0.5, EPSILON);  // -1.0 + 0.5
  ASSERT_NEAR(map_->info.origin.position.y, -1.0, EPSILON);

  // Then try to publish polygon in incorrect frame
  add_shapes_msg->polygons[0].header.frame_id = "incorrect_frame";
  map_.reset();  // Resetting the map to ensure that updated one is received later
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Having incorrect transform, map should not be published
  ASSERT_FALSE(waitMap(timeout));

  vo_server_->stop();
}

TEST_F(Tester, testSwitchDynamicStatic)
{
  setVOServerParams();
  vo_server_->start();

  // Wait for the initial map to be received to not get it in later updates
  const std::chrono::nanoseconds timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(1s * 1.5 / UPDATE_FREQUENCY);
  ASSERT_TRUE(waitMap(timeout));

  // 0.5m shift between polygon and map
  sendTransform(0.5);

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Polygon to be in different frame
  po_msg->header.frame_id = SHAPE_FRAME_ID;
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check that polygon and circle are in correct positions on map
  verifyMap(true, 1.0, 2.5);  // Polygon is moved 0.5m towards to the circle
  // Check that map's origin was updated in accordance to polygon movement
  ASSERT_NEAR(map_->info.origin.position.x, -0.5, EPSILON);  // -1.0 + 0.5
  ASSERT_NEAR(map_->info.origin.position.y, -1.0, EPSILON);

  // Then return to the static model and ensure the everything works correctly
  add_shapes_msg->polygons[0].header.frame_id = GLOBAL_FRAME_ID;
  map_.reset();  // Resetting the map to ensure that updated one is received later
  add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  verifyMap(true);

  vo_server_->stop();
}

// ---------- Corner cases ----------
TEST_F(Tester, switchProcessMap)
{
  setVOServerParams();
  vo_server_->start();

  // Wait for the initial map to be received to not get it in later updates
  const std::chrono::nanoseconds timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(1s * 1.5 / UPDATE_FREQUENCY);
  ASSERT_TRUE(waitMap(timeout));

  // Check that received default map is empty
  ASSERT_EQ(map_->header.frame_id, "map");
  ASSERT_NEAR(map_->info.resolution, 0.1, EPSILON);
  ASSERT_EQ(map_->info.width, 1);
  ASSERT_EQ(map_->info.height, 1);

  // Disable map processing and trying to obtain the map
  vo_server_->setProcessMap(false);

  // Switching map update by calling remove service
  auto remove_shapes_msg = std::make_shared<nav2_msgs::srv::RemoveShapes::Request>();
  remove_shapes_msg->all_objects = true;
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto remove_shapes_result =
    sendRequest<nav2_msgs::srv::RemoveShapes>(remove_shapes_client_, remove_shapes_msg, 2s);
  ASSERT_NE(remove_shapes_result, nullptr);
  ASSERT_TRUE(remove_shapes_result->success);

  // Having process_map_ disabled, map should not be published
  ASSERT_FALSE(waitMap(timeout));

  // Then enable map processing and trying to obtain the map
  vo_server_->setProcessMap(true);

  // Switching map update by calling remove service
  map_.reset();  // Resetting the map to ensure that updated one is received later
  remove_shapes_result =
    sendRequest<nav2_msgs::srv::RemoveShapes>(remove_shapes_client_, remove_shapes_msg, 2s);
  ASSERT_NE(remove_shapes_result, nullptr);
  ASSERT_TRUE(remove_shapes_result->success);

  // Ensure that map is being updated
  ASSERT_TRUE(waitMap(timeout));

  // ... and it is empty
  ASSERT_EQ(map_->header.frame_id, "map");
  ASSERT_NEAR(map_->info.resolution, 0.1, EPSILON);
  ASSERT_EQ(map_->info.width, 1);
  ASSERT_EQ(map_->info.height, 1);

  vo_server_->stop();
}

TEST_F(Tester, testIncorrectMapBoundaries) {
  setVOServerParams();
  vo_server_->start();

  // Set min_x > max_x
  EXPECT_THROW(vo_server_->updateMap(1.0, 0.1, 0.1, 1.0), std::runtime_error);

  // Set min_y > max_y
  EXPECT_THROW(vo_server_->updateMap(0.1, 1.0, 1.0, 0.1), std::runtime_error);
}

TEST_F(Tester, testIncorrectMapBoundariesWhenNoShapes) {
  setVOServerParams();
  vo_server_->start();
  double min_x, min_y, max_x, max_y;
  EXPECT_THROW(vo_server_->getMapBoundaries(min_x, min_y, max_x, max_y), std::runtime_error);
}

TEST_F(Tester, testShapeOutsideMap) {
  setVOServerParams();
  vo_server_->start();

  // Add polygon and circle on map
  auto add_shapes_msg = std::make_shared<nav2_msgs::srv::AddShapes::Request>();
  // Prepare polygon object to add
  auto po_msg = makePolygonObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  // Prepare circle object to add
  auto co_msg = makeCircleObject(
    std::vector<unsigned char>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2});

  add_shapes_msg->polygons.push_back(*po_msg);
  add_shapes_msg->circles.push_back(*co_msg);
  map_.reset();  // Resetting the map to ensure that updated one is received later
  auto add_shapes_result =
    sendRequest<nav2_msgs::srv::AddShapes>(add_shapes_client_, add_shapes_msg, 2s);
  ASSERT_NE(add_shapes_result, nullptr);
  ASSERT_TRUE(add_shapes_result->success);

  // Check the map has been updated correctly
  verifyMap(true);

  // Modify the map to have a shape outside the map
  vo_server_->updateMap(2.0, 2.0, 4.0, 4.0);

  // Try to put the shapes back in the map
  vo_server_->putVectorObjectsOnMap();

  // Verify that map did not corrupted after all false-manipulations
  verifyMap(true);
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
