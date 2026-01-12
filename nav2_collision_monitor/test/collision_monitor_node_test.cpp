// Copyright (c) 2022 Samsung R&D Institute Russia
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
#include <cmath>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/transform_broadcaster.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/collision_monitor_node.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = 1e-5;

static const char BASE_FRAME_ID[]{"base_link"};
static const char SOURCE_FRAME_ID[]{"base_source"};
static const char ODOM_FRAME_ID[]{"odom"};
static const char CMD_VEL_IN_TOPIC[]{"cmd_vel_in"};
static const char CMD_VEL_OUT_TOPIC[]{"cmd_vel_out"};
static const char STATE_TOPIC[]{"collision_monitor_state"};
static const char COLLISION_POINTS_MARKERS_TOPIC[]{"/collision_monitor/collision_points_marker"};
static const char FOOTPRINT_TOPIC[]{"footprint"};
static const char SCAN_NAME[]{"Scan"};
static const char POINTCLOUD_NAME[]{"PointCloud"};
static const char RANGE_NAME[]{"Range"};
static const char POLYGON_NAME[]{"Polygon"};
static const int MIN_POINTS{2};
static const double SLOWDOWN_RATIO{0.7};
static const double LINEAR_LIMIT{0.4};
static const double ANGULAR_LIMIT{0.09};
static const double TIME_BEFORE_COLLISION{1.0};
static const double SIMULATION_TIME_STEP{0.01};
static const double TRANSFORM_TOLERANCE{0.5};
static const double SOURCE_TIMEOUT{5.0};
static const double STOP_PUB_TIMEOUT{0.1};

enum PolygonType
{
  POLYGON_UNKNOWN = 0,
  POLYGON = 1,
  CIRCLE = 2,
  VELOCITY_POLYGON = 3
};

enum SourceType
{
  SOURCE_UNKNOWN = 0,
  SCAN = 1,
  POINTCLOUD = 2,
  RANGE = 3,
  POLYGON_SOURCE = 4
};

enum ActionType
{
  DO_NOTHING = 0,
  STOP = 1,
  SLOWDOWN = 2,
  APPROACH = 3,
  LIMIT = 4,
};

class CollisionMonitorWrapper : public nav2_collision_monitor::CollisionMonitor
{
public:
  void start()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_activate(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  void stop()
  {
    ASSERT_EQ(on_deactivate(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_cleanup(get_current_state()), nav2::CallbackReturn::SUCCESS);
    ASSERT_EQ(on_shutdown(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  void configure()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2::CallbackReturn::SUCCESS);
  }

  void cant_configure()
  {
    ASSERT_EQ(on_configure(get_current_state()), nav2::CallbackReturn::FAILURE);
  }

  bool correctDataReceived(const double expected_dist, const rclcpp::Time & stamp)
  {
    for (std::shared_ptr<nav2_collision_monitor::Source> source : sources_) {
      std::vector<nav2_collision_monitor::Point> collision_points;
      source->getData(stamp, collision_points);
      if (collision_points.size() != 0) {
        const double dist = std::hypot(collision_points[0].x, collision_points[0].y);
        if (std::fabs(dist - expected_dist) <= EPSILON) {
          return true;
        }
      }
    }
    return false;
  }
};  // CollisionMonitorWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();

  // Configuring
  void setCommonParameters();
  void enableSteeringFieldLimiter(
    bool enable, double low_speed_threshold, double high_speed_threshold = 1.0);
  void addPolygon(
    const std::string & polygon_name, const PolygonType type,
    const double size, const std::string & at,
    const std::vector<std::string> & sources_names = std::vector<std::string>());
  void addPolygonVelocitySubPolygon(
    const std::string & polygon_name, const std::string & sub_polygon_name,
    const double linear_min, const double linear_max,
    const double theta_min, const double theta_max,
    const double size);
  void addSource(const std::string & source_name, const SourceType type);
  void setVectors(
    const std::vector<std::string> & polygons,
    const std::vector<std::string> & sources);
  void setPolygonVelocityVectors(
    const std::string & polygon_name,
    const std::vector<std::string> & polygons);

  // Setting TF chains
  void sendTransforms(const rclcpp::Time & stamp);

  // Publish robot footprint
  void publishFootprint(const double radius, const rclcpp::Time & stamp);

  // Main topic/data working routines
  void publishScan(const double dist, const rclcpp::Time & stamp);
  void publishPointCloud(const double dist, const rclcpp::Time & stamp);
  void publishRange(const double dist, const rclcpp::Time & stamp);
  void publishPolygon(const double dist, const rclcpp::Time & stamp);
  void publishCmdVel(const double x, const double y, const double tw);
  bool waitData(
    const double expected_dist,
    const std::chrono::nanoseconds & timeout,
    const rclcpp::Time & stamp);
  bool waitCmdVel(const std::chrono::nanoseconds & timeout);
  bool waitFuture(
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture,
    const std::chrono::nanoseconds & timeout);
  bool waitActionState(const std::chrono::nanoseconds & timeout);
  bool waitCollisionPointsMarker(const std::chrono::nanoseconds & timeout);

protected:
  void cmdVelOutCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  void actionStateCallback(nav2_msgs::msg::CollisionMonitorState::SharedPtr msg);
  void collisionPointsMarkerCallback(visualization_msgs::msg::MarkerArray::SharedPtr msg);

  // CollisionMonitor node
  std::shared_ptr<CollisionMonitorWrapper> cm_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  // Footprint publisher
  nav2::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
    footprint_pub_;

  // Data source publishers
  nav2::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  nav2::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  nav2::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  nav2::Publisher<geometry_msgs::msg::PolygonInstanceStamped>::SharedPtr
    polygon_source_pub_;

  // Working with cmd_vel_in/cmd_vel_out
  nav2::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_pub_;
  nav2::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_sub_;

  geometry_msgs::msg::Twist::SharedPtr cmd_vel_out_;

  // CollisionMonitor Action state
  nav2::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr action_state_sub_;
  nav2_msgs::msg::CollisionMonitorState::SharedPtr action_state_;

  // CollisionMonitor collision points markers
  nav2::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    collision_points_marker_sub_;
  visualization_msgs::msg::MarkerArray::SharedPtr collision_points_marker_msg_;

  // Service client for setting CollisionMonitor parameters
  nav2::ServiceClient<rcl_interfaces::srv::SetParameters>::SharedPtr parameters_client_;
};  // Tester

Tester::Tester()
{
  cm_ = std::make_shared<CollisionMonitorWrapper>();
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(cm_->get_node_base_interface());
  cm_->declare_parameter("enable_stamped_cmd_vel", rclcpp::ParameterValue(false));

  footprint_pub_ = cm_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    FOOTPRINT_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  footprint_pub_->on_activate();

  scan_pub_ = cm_->create_publisher<sensor_msgs::msg::LaserScan>(
    SCAN_NAME, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  scan_pub_->on_activate();
  pointcloud_pub_ = cm_->create_publisher<sensor_msgs::msg::PointCloud2>(
    POINTCLOUD_NAME, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pointcloud_pub_->on_activate();
  range_pub_ = cm_->create_publisher<sensor_msgs::msg::Range>(
    RANGE_NAME, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  range_pub_->on_activate();
  polygon_source_pub_ = cm_->create_publisher<geometry_msgs::msg::PolygonInstanceStamped>(
    POLYGON_NAME, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  polygon_source_pub_->on_activate();

  cmd_vel_in_pub_ = cm_->create_publisher<geometry_msgs::msg::Twist>(
    CMD_VEL_IN_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  cmd_vel_in_pub_->on_activate();
  cmd_vel_out_sub_ = cm_->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_OUT_TOPIC,
    std::bind(&Tester::cmdVelOutCallback, this, std::placeholders::_1));

  action_state_sub_ = cm_->create_subscription<nav2_msgs::msg::CollisionMonitorState>(
    STATE_TOPIC,
    std::bind(&Tester::actionStateCallback, this, std::placeholders::_1));
  collision_points_marker_sub_ = cm_->create_subscription<visualization_msgs::msg::MarkerArray>(
    COLLISION_POINTS_MARKERS_TOPIC,
    std::bind(&Tester::collisionPointsMarkerCallback, this, std::placeholders::_1));
  parameters_client_ =
    cm_->create_client<rcl_interfaces::srv::SetParameters>(
    std::string(
      cm_->get_name()) + "/set_parameters");
}

Tester::~Tester()
{
  footprint_pub_.reset();

  scan_pub_->on_deactivate();
  scan_pub_.reset();
  pointcloud_pub_->on_deactivate();
  pointcloud_pub_.reset();
  range_pub_->on_deactivate();
  range_pub_.reset();
  polygon_source_pub_->on_deactivate();
  polygon_source_pub_.reset();

  cmd_vel_in_pub_->on_deactivate();
  cmd_vel_in_pub_.reset();
  cmd_vel_out_sub_.reset();

  action_state_sub_.reset();
  collision_points_marker_sub_.reset();

  cm_.reset();
  executor_.reset();
}

void Tester::setCommonParameters()
{
  cm_->declare_parameter(
    "cmd_vel_in_topic", rclcpp::ParameterValue(CMD_VEL_IN_TOPIC));
  cm_->set_parameter(
    rclcpp::Parameter("cmd_vel_in_topic", CMD_VEL_IN_TOPIC));
  cm_->declare_parameter(
    "cmd_vel_out_topic", rclcpp::ParameterValue(CMD_VEL_OUT_TOPIC));
  cm_->set_parameter(
    rclcpp::Parameter("cmd_vel_out_topic", CMD_VEL_OUT_TOPIC));
  cm_->declare_parameter(
    "state_topic", rclcpp::ParameterValue(STATE_TOPIC));
  cm_->set_parameter(
    rclcpp::Parameter("state_topic", STATE_TOPIC));

  cm_->declare_parameter(
    "base_frame_id", rclcpp::ParameterValue(BASE_FRAME_ID));
  cm_->set_parameter(
    rclcpp::Parameter("base_frame_id", BASE_FRAME_ID));
  cm_->declare_parameter(
    "odom_frame_id", rclcpp::ParameterValue(ODOM_FRAME_ID));
  cm_->set_parameter(
    rclcpp::Parameter("odom_frame_id", ODOM_FRAME_ID));

  cm_->declare_parameter(
    "transform_tolerance", rclcpp::ParameterValue(TRANSFORM_TOLERANCE));
  cm_->set_parameter(
    rclcpp::Parameter("transform_tolerance", TRANSFORM_TOLERANCE));
  cm_->declare_parameter(
    "source_timeout", rclcpp::ParameterValue(SOURCE_TIMEOUT));
  cm_->set_parameter(
    rclcpp::Parameter("source_timeout", SOURCE_TIMEOUT));

  cm_->declare_parameter(
    "stop_pub_timeout", rclcpp::ParameterValue(STOP_PUB_TIMEOUT));
  cm_->set_parameter(
    rclcpp::Parameter("stop_pub_timeout", STOP_PUB_TIMEOUT));

  // Steering field limiter parameters (disabled by default)
  cm_->declare_parameter(
    "steering_field_limiter_enabled", rclcpp::ParameterValue(false));
  cm_->set_parameter(
    rclcpp::Parameter("steering_field_limiter_enabled", false));

  cm_->declare_parameter(
    "low_speed_threshold", rclcpp::ParameterValue(0.3));
  cm_->set_parameter(
    rclcpp::Parameter("low_speed_threshold", 0.3));

  cm_->declare_parameter(
    "high_speed_threshold", rclcpp::ParameterValue(1.0));
  cm_->set_parameter(
    rclcpp::Parameter("high_speed_threshold", 1.0));
}

void Tester::enableSteeringFieldLimiter(
  bool enable, double low_speed_threshold, double high_speed_threshold)
{
  // Parameters are already declared in setCommonParameters(), just set values
  cm_->set_parameter(
    rclcpp::Parameter("steering_field_limiter_enabled", enable));
  cm_->set_parameter(
    rclcpp::Parameter("low_speed_threshold", low_speed_threshold));
  cm_->set_parameter(
    rclcpp::Parameter("high_speed_threshold", high_speed_threshold));
}

void Tester::addPolygon(
  const std::string & polygon_name, const PolygonType type,
  const double size, const std::string & at,
  const std::vector<std::string> & sources_names)
{
  if (type == POLYGON) {
    cm_->declare_parameter(
      polygon_name + ".type", rclcpp::ParameterValue("polygon"));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".type", "polygon"));

    if (at != "approach") {
      const std::string points = "[[" +
        std::to_string(size) + ", " + std::to_string(size) + "], [" +
        std::to_string(size) + ", " + std::to_string(-size) + "], [" +
        std::to_string(-size) + ", " + std::to_string(-size) + "], [" +
        std::to_string(-size) + ", " + std::to_string(size) + "]]";
      cm_->declare_parameter(
        polygon_name + ".points", rclcpp::ParameterValue(points));
      cm_->set_parameter(
        rclcpp::Parameter(polygon_name + ".points", points));
    } else {  // at == "approach"
      cm_->declare_parameter(
        polygon_name + ".footprint_topic", rclcpp::ParameterValue(FOOTPRINT_TOPIC));
      cm_->set_parameter(
        rclcpp::Parameter(polygon_name + ".footprint_topic", FOOTPRINT_TOPIC));
    }
  } else if (type == CIRCLE) {
    cm_->declare_parameter(
      polygon_name + ".type", rclcpp::ParameterValue("circle"));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".type", "circle"));

    cm_->declare_parameter(
      polygon_name + ".radius", rclcpp::ParameterValue(size));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".radius", size));
  } else if (type == VELOCITY_POLYGON) {
    cm_->declare_parameter(
      polygon_name + ".type", rclcpp::ParameterValue("velocity_polygon"));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".type", "velocity_polygon"));
    cm_->declare_parameter(
      polygon_name + ".holonomic", rclcpp::ParameterValue(false));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".holonomic", false));
  } else {  // type == POLYGON_UNKNOWN
    cm_->declare_parameter(
      polygon_name + ".type", rclcpp::ParameterValue("unknown"));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".type", "unknown"));
  }

  cm_->declare_parameter(
    polygon_name + ".enabled", rclcpp::ParameterValue(true));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".enabled", true));

  cm_->declare_parameter(
    polygon_name + ".action_type", rclcpp::ParameterValue(at));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".action_type", at));

  cm_->declare_parameter(
    polygon_name + ".min_points", rclcpp::ParameterValue(MIN_POINTS));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".min_points", MIN_POINTS));

  cm_->declare_parameter(
    polygon_name + ".slowdown_ratio", rclcpp::ParameterValue(SLOWDOWN_RATIO));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".slowdown_ratio", SLOWDOWN_RATIO));

  cm_->declare_parameter(
    polygon_name + ".linear_limit", rclcpp::ParameterValue(LINEAR_LIMIT));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".linear_limit", LINEAR_LIMIT));

  cm_->declare_parameter(
    polygon_name + ".angular_limit", rclcpp::ParameterValue(ANGULAR_LIMIT));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".angular_limit", ANGULAR_LIMIT));

  cm_->declare_parameter(
    polygon_name + ".time_before_collision", rclcpp::ParameterValue(TIME_BEFORE_COLLISION));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".time_before_collision", TIME_BEFORE_COLLISION));

  cm_->declare_parameter(
    polygon_name + ".simulation_time_step", rclcpp::ParameterValue(SIMULATION_TIME_STEP));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".simulation_time_step", SIMULATION_TIME_STEP));

  cm_->declare_parameter(
    polygon_name + ".visualize", rclcpp::ParameterValue(false));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".visualize", false));

  cm_->declare_parameter(
    polygon_name + ".polygon_pub_topic", rclcpp::ParameterValue(polygon_name));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + ".polygon_pub_topic", polygon_name));

  if (!sources_names.empty()) {
    cm_->declare_parameter(
      polygon_name + ".sources_names", rclcpp::ParameterValue(sources_names));
    cm_->set_parameter(
      rclcpp::Parameter(polygon_name + ".sources_names", sources_names));
  }
}

void Tester::addPolygonVelocitySubPolygon(
  const std::string & polygon_name, const std::string & sub_polygon_name,
  const double linear_min, const double linear_max,
  const double theta_min, const double theta_max,
  const double size)
{
  const std::string points = "[[" +
    std::to_string(size) + ", " + std::to_string(size) + "], [" +
    std::to_string(size) + ", " + std::to_string(-size) + "], [" +
    std::to_string(-size) + ", " + std::to_string(-size) + "], [" +
    std::to_string(-size) + ", " + std::to_string(size) + "]]";
  cm_->declare_parameter(
    polygon_name + "." + sub_polygon_name + ".points", rclcpp::ParameterValue(points));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + "." + sub_polygon_name + ".points", points));

  cm_->declare_parameter(
    polygon_name + "." + sub_polygon_name + ".linear_min", rclcpp::ParameterValue(linear_min));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + "." + sub_polygon_name + ".linear_min", linear_min));

  cm_->declare_parameter(
    polygon_name + "." + sub_polygon_name + ".linear_max", rclcpp::ParameterValue(linear_max));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + "." + sub_polygon_name + ".linear_max", linear_max));

  cm_->declare_parameter(
    polygon_name + "." + sub_polygon_name + ".theta_min", rclcpp::ParameterValue(theta_min));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + "." + sub_polygon_name + ".theta_min", theta_min));

  cm_->declare_parameter(
    polygon_name + "." + sub_polygon_name + ".theta_max", rclcpp::ParameterValue(theta_max));
  cm_->set_parameter(
    rclcpp::Parameter(polygon_name + "." + sub_polygon_name + ".theta_max", theta_max));
}

void Tester::addSource(
  const std::string & source_name, const SourceType type)
{
  if (type == SCAN) {
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("scan"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "scan"));
  } else if (type == POINTCLOUD) {
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("pointcloud"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "pointcloud"));

    cm_->declare_parameter(
      source_name + ".min_height", rclcpp::ParameterValue(0.1));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".min_height", 0.1));
    cm_->declare_parameter(
      source_name + ".max_height", rclcpp::ParameterValue(1.0));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".max_height", 1.0));
  } else if (type == RANGE) {
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("range"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "range"));

    cm_->declare_parameter(
      source_name + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 200));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".obstacles_angle", M_PI / 200));
  } else if (type == POLYGON_SOURCE) {
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("polygon"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "polygon"));

    cm_->declare_parameter(
      source_name + ".sampling_distance", rclcpp::ParameterValue(0.1));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".sampling_distance", 0.1));
    cm_->declare_parameter(
      source_name + ".polygon_similarity_threshold", rclcpp::ParameterValue(2.0));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".polygon_similarity_threshold", 2.0));
  } else {  // type == SOURCE_UNKNOWN
    cm_->declare_parameter(
      source_name + ".type", rclcpp::ParameterValue("unknown"));
    cm_->set_parameter(
      rclcpp::Parameter(source_name + ".type", "unknown"));
  }

  cm_->declare_parameter(
    source_name + ".topic", rclcpp::ParameterValue(source_name));
  cm_->set_parameter(
    rclcpp::Parameter(source_name + ".topic", source_name));
}

void Tester::setVectors(
  const std::vector<std::string> & polygons,
  const std::vector<std::string> & sources)
{
  cm_->declare_parameter("polygons", rclcpp::ParameterValue(polygons));
  cm_->set_parameter(rclcpp::Parameter("polygons", polygons));

  cm_->declare_parameter("observation_sources", rclcpp::ParameterValue(sources));
  cm_->set_parameter(rclcpp::Parameter("observation_sources", sources));
}

void Tester::setPolygonVelocityVectors(
  const std::string & polygon_name,
  const std::vector<std::string> & polygons)
{
  cm_->declare_parameter(polygon_name + ".velocity_polygons", rclcpp::ParameterValue(polygons));
  cm_->set_parameter(rclcpp::Parameter(polygon_name + ".velocity_polygons", polygons));
}

void Tester::sendTransforms(const rclcpp::Time & stamp)
{
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(cm_);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  // Fill TF buffer ahead for future transform usage in CollisionMonitor::process()
  const rclcpp::Duration ahead = 1000ms;
  for (rclcpp::Time t = stamp; t <= stamp + ahead; t += rclcpp::Duration(50ms)) {
    transform.header.stamp = t;

    // base_frame -> source_frame transform
    transform.header.frame_id = BASE_FRAME_ID;
    transform.child_frame_id = SOURCE_FRAME_ID;
    tf_broadcaster->sendTransform(transform);

    // odom_frame -> base_frame transform
    transform.header.frame_id = ODOM_FRAME_ID;
    transform.child_frame_id = BASE_FRAME_ID;
    tf_broadcaster->sendTransform(transform);
  }
}

void Tester::publishFootprint(const double radius, const rclcpp::Time & stamp)
{
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> msg =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();

  msg->header.frame_id = BASE_FRAME_ID;
  msg->header.stamp = stamp;

  geometry_msgs::msg::Point32 p;
  p.x = radius;
  p.y = radius;
  msg->polygon.points.push_back(p);
  p.x = radius;
  p.y = -radius;
  msg->polygon.points.push_back(p);
  p.x = -radius;
  p.y = -radius;
  msg->polygon.points.push_back(p);
  p.x = -radius;
  p.y = radius;
  msg->polygon.points.push_back(p);

  footprint_pub_->publish(std::move(msg));
}

void Tester::publishScan(const double dist, const rclcpp::Time & stamp)
{
  std::unique_ptr<sensor_msgs::msg::LaserScan> msg =
    std::make_unique<sensor_msgs::msg::LaserScan>();

  msg->header.frame_id = SOURCE_FRAME_ID;
  msg->header.stamp = stamp;

  msg->angle_min = 0.0;
  msg->angle_max = 2 * M_PI;
  msg->angle_increment = M_PI / 180;
  msg->time_increment = 0.0;
  msg->scan_time = 0.0;
  msg->range_min = 0.1;
  msg->range_max = dist + 1.0;
  std::vector<float> ranges(360, dist);
  msg->ranges = ranges;

  scan_pub_->publish(std::move(msg));
}

void Tester::publishPointCloud(const double dist, const rclcpp::Time & stamp)
{
  std::unique_ptr<sensor_msgs::msg::PointCloud2> msg =
    std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*msg);

  msg->header.frame_id = SOURCE_FRAME_ID;
  msg->header.stamp = stamp;

  modifier.setPointCloud2Fields(
    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

  // Point 0: (dist, 0.01, 0.2)
  *iter_x = dist;
  *iter_y = 0.01;
  *iter_z = 0.2;
  ++iter_x; ++iter_y; ++iter_z;

  // Point 1: (dist, -0.01, 0.2)
  *iter_x = dist;
  *iter_y = -0.01;
  *iter_z = 0.2;

  pointcloud_pub_->publish(std::move(msg));
}

void Tester::publishRange(const double dist, const rclcpp::Time & stamp)
{
  std::unique_ptr<sensor_msgs::msg::Range> msg =
    std::make_unique<sensor_msgs::msg::Range>();

  msg->header.frame_id = SOURCE_FRAME_ID;
  msg->header.stamp = stamp;

  msg->radiation_type = 0;
  msg->field_of_view = M_PI / 10;
  msg->min_range = 0.1;
  msg->max_range = dist + 0.1;
  msg->range = dist;

  range_pub_->publish(std::move(msg));
}

void Tester::publishPolygon(const double dist, const rclcpp::Time & stamp)
{
  std::unique_ptr<geometry_msgs::msg::PolygonInstanceStamped> msg =
    std::make_unique<geometry_msgs::msg::PolygonInstanceStamped>();

  msg->header.frame_id = SOURCE_FRAME_ID;
  msg->header.stamp = stamp;

  geometry_msgs::msg::Point32 p;
  p.x = 1.0;
  p.y = dist;
  msg->polygon.polygon.points.push_back(p);
  p.x = -1.0;
  p.y = dist;
  msg->polygon.polygon.points.push_back(p);
  p.x = -1.0;
  p.y = dist + 1.0;
  msg->polygon.polygon.points.push_back(p);
  p.x = 1.0;
  p.y = dist + 1.0;
  msg->polygon.polygon.points.push_back(p);

  polygon_source_pub_->publish(std::move(msg));
}

void Tester::publishCmdVel(const double x, const double y, const double tw)
{
  // Reset cmd_vel_out_ before calling CollisionMonitor::process()
  cmd_vel_out_ = nullptr;
  action_state_ = nullptr;
  collision_points_marker_msg_ = nullptr;

  std::unique_ptr<geometry_msgs::msg::Twist> msg =
    std::make_unique<geometry_msgs::msg::Twist>();

  msg->linear.x = x;
  msg->linear.y = y;
  msg->angular.z = tw;

  cmd_vel_in_pub_->publish(std::move(msg));
}

bool Tester::waitData(
  const double expected_dist,
  const std::chrono::nanoseconds & timeout,
  const rclcpp::Time & stamp)
{
  rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (cm_->correctDataReceived(expected_dist, stamp)) {
      return true;
    }
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitCmdVel(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (cmd_vel_out_) {
      return true;
    }
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitFuture(
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture result_future,
  const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    std::future_status status = result_future.wait_for(10ms);
    if (status == std::future_status::ready) {
      return true;
    }
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitActionState(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (action_state_) {
      return true;
    }
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitCollisionPointsMarker(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = cm_->now();
  while (rclcpp::ok() && cm_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (collision_points_marker_msg_) {
      return true;
    }
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void Tester::cmdVelOutCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_out_ = msg;
}

void Tester::actionStateCallback(nav2_msgs::msg::CollisionMonitorState::SharedPtr msg)
{
  action_state_ = msg;
}

void Tester::collisionPointsMarkerCallback(visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  collision_points_marker_msg_ = msg;
}

TEST_F(Tester, testProcessStopSlowdownLimit)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner for robot stop.
  setCommonParameters();
  addPolygon("Limit", POLYGON, 3.0, "limit");
  addPolygon("SlowDown", POLYGON, 2.0, "slowdown");
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Limit", "SlowDown", "Stop"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is far away from robot
  publishScan(4.5, curr_time);
  ASSERT_TRUE(waitData(4.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);

  // 2. Obstacle is in limit robot zone
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  const double speed = std::sqrt(0.5 * 0.5 + 0.2 * 0.2);
  const double linear_ratio = LINEAR_LIMIT / speed;
  const double angular_ratio = ANGULAR_LIMIT / 0.1;
  const double ratio = std::min(linear_ratio, angular_ratio);
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5 * ratio, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2 * ratio, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1 * ratio, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, LIMIT);
  ASSERT_EQ(action_state_->polygon_name, "Limit");

  // 3. Obstacle is in slowdown robot zone
  publishScan(1.5, curr_time);
  ASSERT_TRUE(waitData(1.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5 * SLOWDOWN_RATIO, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2 * SLOWDOWN_RATIO, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1 * SLOWDOWN_RATIO, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  ASSERT_EQ(action_state_->polygon_name, "SlowDown");

  // 4. Obstacle is inside stop zone
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_EQ(action_state_->polygon_name, "Stop");

  // 5. Restoring back normal operation
  publishScan(4.5, curr_time);
  ASSERT_TRUE(waitData(4.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testPolygonSource)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner for robot stop.
  setCommonParameters();
  // Set source_timeout to 0.0 to clear out quickly the polygons from test to test
  cm_->set_parameter(
    rclcpp::Parameter("source_timeout", 0.1));
  addPolygon("Limit", POLYGON, 3.0, "limit");
  addPolygon("SlowDown", POLYGON, 2.0, "slowdown");
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(POLYGON_NAME, POLYGON_SOURCE);
  setVectors({"Limit", "SlowDown", "Stop"}, {POLYGON_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is far away from robot
  publishPolygon(4.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.0, 4.5), 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);

  // 2. Obstacle is in limit robot zone
  publishPolygon(3.0, curr_time);
  EXPECT_TRUE(waitData(std::hypot(1.0, 3.0), 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  EXPECT_TRUE(waitCmdVel(500ms));
  const double speed = std::sqrt(0.5 * 0.5 + 0.2 * 0.2);
  const double linear_ratio = LINEAR_LIMIT / speed;
  const double angular_ratio = ANGULAR_LIMIT / 0.1;
  const double ratio = std::min(linear_ratio, angular_ratio);
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.5 * ratio, EPSILON);
  EXPECT_NEAR(cmd_vel_out_->linear.y, 0.2 * ratio, EPSILON);
  EXPECT_NEAR(cmd_vel_out_->angular.z, 0.1 * ratio, EPSILON);
  EXPECT_TRUE(waitActionState(500ms));
  EXPECT_EQ(action_state_->action_type, LIMIT);
  EXPECT_EQ(action_state_->polygon_name, "Limit");

  // 3. Obstacle is in slowdown robot zone
  publishPolygon(1.5, curr_time);
  EXPECT_TRUE(waitData(std::hypot(1.0, 1.5), 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  EXPECT_TRUE(waitCmdVel(500ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.5 * SLOWDOWN_RATIO, EPSILON);
  EXPECT_NEAR(cmd_vel_out_->linear.y, 0.2 * SLOWDOWN_RATIO, EPSILON);
  EXPECT_NEAR(cmd_vel_out_->angular.z, 0.1 * SLOWDOWN_RATIO, EPSILON);
  EXPECT_TRUE(waitActionState(500ms));
  EXPECT_EQ(action_state_->action_type, SLOWDOWN);
  EXPECT_EQ(action_state_->polygon_name, "SlowDown");

  // 4. Obstacle is inside stop zone
  curr_time = cm_->now();
  publishPolygon(0.5, curr_time);
  EXPECT_TRUE(waitData(std::hypot(1.0, 0.5), 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  EXPECT_TRUE(waitCmdVel(500ms));
  EXPECT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  EXPECT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  EXPECT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  EXPECT_TRUE(waitActionState(500ms));
  EXPECT_EQ(action_state_->action_type, STOP);
  EXPECT_EQ(action_state_->polygon_name, "Stop");

  // 5. Restoring back normal operation
  publishPolygon(4.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.0, 4.5), 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testProcessApproach)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making one circle footprint for approach.
  setCommonParameters();
  addPolygon("Approach", CIRCLE, 1.0, "approach");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Approach"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is far away from robot
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 2. Approaching obstacle (0.2 m ahead from robot footprint)
  publishScan(1.2, curr_time);
  ASSERT_TRUE(waitData(1.2, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // change_ratio = (0.2 m / speed m/s) / TIME_BEFORE_COLLISION s
  // where speed = sqrt(0.5^2 + 0.2^2) ~= 0.538516481 m/s
  const double change_ratio = (0.2 / 0.538516481) / TIME_BEFORE_COLLISION;
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 0.5 * change_ratio, 0.5 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(
    cmd_vel_out_->linear.y, 0.2 * change_ratio, 0.2 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, APPROACH);
  ASSERT_EQ(action_state_->polygon_name, "Approach");

  // 3. Obstacle is inside robot footprint
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  // Publish impossible cmd_vel to ensure robot footprint is checked
  publishCmdVel(1000000000.0, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 4. Restoring back normal operation
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testProcessApproachRotation)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making one circle footprint for approach.
  setCommonParameters();
  addPolygon("Approach", POLYGON, 1.0, "approach");
  addSource(RANGE_NAME, RANGE);
  setVectors({"Approach"}, {RANGE_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Publish robot footprint
  publishFootprint(1.0, curr_time);

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is far away from robot
  publishRange(2.0, curr_time);
  ASSERT_TRUE(waitData(2.0, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, M_PI / 4, EPSILON);

  // 2. Approaching rotation to obstacle ( M_PI / 4 - M_PI / 20 ahead from robot)
  publishRange(1.4, curr_time);
  ASSERT_TRUE(waitData(1.4, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(
    cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(
    cmd_vel_out_->angular.z,
    M_PI / 5,
    (M_PI / 4) * (SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION));
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, APPROACH);
  ASSERT_EQ(action_state_->polygon_name, "Approach");

  // 3. Obstacle is inside robot footprint
  publishRange(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // 4. Restoring back normal operation
  publishRange(2.5, curr_time);
  ASSERT_TRUE(waitData(2.5, 500ms, curr_time));
  publishCmdVel(0.0, 0.0, M_PI / 4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, M_PI / 4, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testCrossOver)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner circle
  // as robot footprint for approach.
  setCommonParameters();
  addPolygon("SlowDown", POLYGON, 2.0, "slowdown");
  addPolygon("Approach", CIRCLE, 1.0, "approach");
  addSource(POINTCLOUD_NAME, POINTCLOUD);
  addSource(RANGE_NAME, RANGE);
  setVectors({"SlowDown", "Approach"}, {POINTCLOUD_NAME, RANGE_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is not in the slowdown zone, but less than TIME_BEFORE_COLLISION (ahead in 1.5 m).
  // Robot should approach the obstacle.
  publishPointCloud(2.5, curr_time);
  publishRange(2.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(2.5, 0.01), 500ms, curr_time));
  publishCmdVel(3.0, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // change_ratio = (1.5 m / 3.0 m/s) / TIME_BEFORE_COLLISION s
  double change_ratio = (1.5 / 3.0) / TIME_BEFORE_COLLISION;
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 3.0 * change_ratio, 3.0 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, APPROACH);
  ASSERT_EQ(action_state_->polygon_name, "Approach");

  // 2. Obstacle is inside slowdown zone, but speed is too slow for approach
  publishRange(1.5, curr_time);
  ASSERT_TRUE(waitData(1.5, 500ms, curr_time));
  publishCmdVel(0.1, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.1 * SLOWDOWN_RATIO, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  ASSERT_EQ(action_state_->polygon_name, "SlowDown");

  // 3. Increase robot speed to return again into approach mode.
  // The speed should be safer for approach mode, so robot will go to the approach (ahead in 0.5 m)
  // even while it is already inside slowdown area.
  publishCmdVel(1.0, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // change_ratio = (0.5 m / 1.0 m/s) / TIME_BEFORE_COLLISION s
  change_ratio = (0.5 / 1.0) / TIME_BEFORE_COLLISION;
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 1.0 * change_ratio, 1.0 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, APPROACH);
  ASSERT_EQ(action_state_->polygon_name, "Approach");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testSourceTimeout)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner circle
  // as robot footprint for approach.
  setCommonParameters();
  addPolygon("SlowDown", POLYGON, 2.0, "slowdown");
  addPolygon("Approach", CIRCLE, 1.0, "approach");
  addSource(POINTCLOUD_NAME, POINTCLOUD);
  addSource(RANGE_NAME, RANGE);
  setVectors({"SlowDown", "Approach"}, {POINTCLOUD_NAME, RANGE_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // Obstacle is not in the slowdown zone, but less than TIME_BEFORE_COLLISION (ahead in 1.5 m).
  // Robot should approach the obstacle.
  publishPointCloud(2.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(2.5, 0.01), 500ms, curr_time));
  publishCmdVel(3.0, 3.0, 3.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // Range configured but not published, range source should be considered invalid
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_EQ(action_state_->polygon_name, "invalid source");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testSourceTimeoutOverride)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner circle
  // as robot footprint for approach.
  setCommonParameters();
  addPolygon("SlowDown", POLYGON, 2.0, "slowdown");
  addPolygon("Approach", CIRCLE, 1.0, "approach");
  addSource(POINTCLOUD_NAME, POINTCLOUD);
  addSource(RANGE_NAME, RANGE);
  setVectors({"SlowDown", "Approach"}, {POINTCLOUD_NAME, RANGE_NAME});
  cm_->set_parameter(rclcpp::Parameter("source_timeout", 0.0));

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // Obstacle is not in the slowdown zone, but less than TIME_BEFORE_COLLISION (ahead in 1.5 m).
  // Robot should approach the obstacle.
  publishPointCloud(2.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(2.5, 0.01), 500ms, curr_time));
  publishCmdVel(3.0, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // change_ratio = (1.5 m / 3.0 m/s) / TIME_BEFORE_COLLISION s
  double change_ratio = (1.5 / 3.0) / TIME_BEFORE_COLLISION;
  // Range configured but not published, range source should be considered invalid
  // but as we set the source_timeout of the Range source to 0.0, its validity check is overridden
  ASSERT_NEAR(
    cmd_vel_out_->linear.x, 3.0 * change_ratio, 3.0 * SIMULATION_TIME_STEP / TIME_BEFORE_COLLISION);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, APPROACH);
  ASSERT_EQ(action_state_->polygon_name, "Approach");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testCeasePublishZeroVel)
{
  rclcpp::Time curr_time = cm_->now();

  // Configure stop and approach zones, and basic data source
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addPolygon("Approach", CIRCLE, 2.0, "approach");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Stop", "Approach"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // 1. Obstacle is inside approach footprint: robot should stop
  publishScan(1.5, curr_time);
  ASSERT_TRUE(waitData(1.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, APPROACH);
  ASSERT_EQ(action_state_->polygon_name, "Approach");

  // Wait more than STOP_PUB_TIMEOUT time
  std::this_thread::sleep_for(std::chrono::duration<double>(STOP_PUB_TIMEOUT + 0.01));

  // 2. Check that zero cmd_vel_out velocity won't be published more for this case
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_FALSE(waitCmdVel(100ms));

  // 3. Release robot to normal operation
  publishScan(3.0, curr_time);
  ASSERT_TRUE(waitData(3.0, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // 4. Obstacle is inside stop zone
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_EQ(action_state_->polygon_name, "Stop");

  // Wait more than STOP_PUB_TIMEOUT time
  std::this_thread::sleep_for(std::chrono::duration<double>(STOP_PUB_TIMEOUT + 0.01));

  // 5. Check that zero cmd_vel_out velocity won't be published more for this case
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_FALSE(waitCmdVel(100ms));

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testPolygonNotEnabled)
{
  // Set Collision Monitor parameters.
  // Create a STOP polygon
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  // Create a Scan source
  addSource(SCAN_NAME, SCAN);
  setVectors({"Stop"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Check that robot stops when polygon is enabled
  rclcpp::Time curr_time = cm_->now();
  sendTransforms(curr_time);
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_EQ(action_state_->polygon_name, "Stop");

  // Disable polygon by calling service
  auto set_parameters_msg = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  auto parameter_msg = std::make_shared<rcl_interfaces::msg::Parameter>();
  parameter_msg->name = "Stop.enabled";
  parameter_msg->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  parameter_msg->value.bool_value = false;
  set_parameters_msg->parameters.push_back(*parameter_msg);
  auto result_future = parameters_client_->async_call(set_parameters_msg);
  ASSERT_TRUE(waitFuture(result_future, 2s));

  // Check that robot does not stop when polygon is disabled
  curr_time = cm_->now();
  sendTransforms(curr_time);
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testSourceNotEnabled)
{
  // Set Collision Monitor parameters.
  // Create a STOP polygon
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  // Create a Scan source
  addSource(SCAN_NAME, SCAN);
  setVectors({"Stop"}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Check that robot stops when source is enabled
  rclcpp::Time curr_time = cm_->now();
  sendTransforms(curr_time);
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_EQ(action_state_->polygon_name, "Stop");

  // Disable source by calling service
  auto set_parameters_msg = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  auto parameter_msg = std::make_shared<rcl_interfaces::msg::Parameter>();
  parameter_msg->name = std::string(SCAN_NAME) + ".enabled";
  parameter_msg->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  parameter_msg->value.bool_value = false;
  set_parameters_msg->parameters.push_back(*parameter_msg);
  auto result_future = parameters_client_->async_call(set_parameters_msg);
  ASSERT_TRUE(waitFuture(result_future, 2s));

  // Check that robot does not stop when source is disabled
  curr_time = cm_->now();
  sendTransforms(curr_time);
  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.2, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testProcessNonActive)
{
  rclcpp::Time curr_time = cm_->now();

  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"Stop"}, {SCAN_NAME});

  // Configure Collision Monitor node, but not activate
  cm_->configure();

  // Share TF
  sendTransforms(curr_time);

  // Call CollisionMonitor::process()
  publishCmdVel(1.0, 0.0, 0.0);
  // ... and check that cmd_vel_out was not published
  ASSERT_FALSE(waitCmdVel(100ms));

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testIncorrectPolygonType)
{
  setCommonParameters();
  addPolygon("UnknownShape", POLYGON_UNKNOWN, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  setVectors({"UnknownShape"}, {SCAN_NAME});

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testIncorrectSourceType)
{
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource("UnknownSource", SOURCE_UNKNOWN);
  setVectors({"Stop"}, {"UnknownSource"});

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testPolygonsNotSet)
{
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testSourcesNotSet)
{
  setCommonParameters();
  addPolygon("Stop", POLYGON, 1.0, "stop");
  addSource(SCAN_NAME, SCAN);
  cm_->declare_parameter("polygons", rclcpp::ParameterValue(std::vector<std::string>{"Stop"}));
  cm_->set_parameter(rclcpp::Parameter("polygons", std::vector<std::string>{"Stop"}));

  // Check that Collision Monitor node can not be configured for this parameters set
  cm_->cant_configure();
}

TEST_F(Tester, testCollisionPointsMarkers)
{
  rclcpp::Time curr_time = cm_->now();

  // Set Collision Monitor parameters.
  // Making two polygons: outer polygon for slowdown and inner for robot stop.
  setCommonParameters();
  addSource(SCAN_NAME, SCAN);
  setVectors({}, {SCAN_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  sendTransforms(curr_time);

  // No source published, empty marker array published
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCollisionPointsMarker(500ms));
  ASSERT_EQ(collision_points_marker_msg_->markers.size(), 0u);

  publishScan(0.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));
  publishCmdVel(0.5, 0.2, 0.1);
  ASSERT_TRUE(waitCollisionPointsMarker(500ms));
  ASSERT_NE(collision_points_marker_msg_->markers[0].points.size(), 0u);
  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testVelocityPolygonStop)
{
  // Set Collision Monitor parameters.
  // Add velocity polygon with 2 sub polygon:
  // 1. Forward:  0 -> 0.5 m/s
  // 2. Backward: 0 -> -0.5 m/s
  setCommonParameters();
  addPolygon("VelocityPolygon", VELOCITY_POLYGON, 1.0, "stop");
  addPolygonVelocitySubPolygon("VelocityPolygon", "Forward", 0.0, 0.5, 0.0, 1.0, 4.0);
  addPolygonVelocitySubPolygon("VelocityPolygon", "Backward", -0.5, 0.0, 0.0, 1.0, 2.0);
  setPolygonVelocityVectors("VelocityPolygon", {"Forward", "Backward"});
  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"VelocityPolygon"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  // Start Collision Monitor node
  cm_->start();
  // Check that robot stops when source is enabled
  sendTransforms(curr_time);

  // 1. Obstacle is far away from Forward velocity polygon
  publishPointCloud(4.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(4.5, 0.01), 500ms, curr_time));
  publishCmdVel(0.4, 0.0, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.4, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);

  // 2. Obstacle is in Forward velocity polygon
  publishPointCloud(3.0, curr_time);
  ASSERT_TRUE(waitData(std::hypot(3.0, 0.01), 500ms, curr_time));
  publishCmdVel(0.4, 0.0, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_EQ(action_state_->polygon_name, "VelocityPolygon");

  // 3. Switch to Backward velocity polygon
  // Obstacle is far away from Backward velocity polygon
  publishCmdVel(-0.4, 0.0, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, -0.4, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.1, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, DO_NOTHING);
  ASSERT_EQ(action_state_->polygon_name, "");

  // 4. Obstacle is in Backward velocity polygon
  publishPointCloud(-1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(-1.5, 0.01), 500ms, curr_time));
  publishCmdVel(-0.4, 0.0, 0.1);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->linear.y, 0.0, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_EQ(action_state_->polygon_name, "VelocityPolygon");

  // Stop Collision Monitor node
  cm_->stop();
}

TEST_F(Tester, testSourceAssociatedToPolygon)
{
  // Set Collision Monitor parameters:
  // - 2 sources (scan and range)
  // - 1 stop polygon associated to range source
  // - 1 slowdown polygon (associated with all sources by default)
  setCommonParameters();
  addSource(SCAN_NAME, SCAN);
  addSource(RANGE_NAME, RANGE);
  std::vector<std::string> range_only_sources_names = {RANGE_NAME};
  std::vector<std::string> all_sources_names = {SCAN_NAME, RANGE_NAME};
  addPolygon("StopOnRangeSource", POLYGON, 1.0, "stop", range_only_sources_names);
  addPolygon("SlowdownOnAllSources", POLYGON, 1.0, "slowdown");
  setVectors({"StopOnRangeSource", "SlowdownOnAllSources"}, {SCAN_NAME, RANGE_NAME});

  // Start Collision Monitor node
  cm_->start();

  // Share TF
  rclcpp::Time curr_time = cm_->now();
  sendTransforms(curr_time);

  // Publish sources so that :
  // - scan obstacle is in polygons
  // - range obstacle is far away from polygons
  publishScan(0.5, curr_time);
  publishRange(4.5, curr_time);
  ASSERT_TRUE(waitData(0.5, 500ms, curr_time));

  // Publish cmd vel
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));

  // Since the stop polygon is only checking range source, slowdown action should be applied
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  ASSERT_EQ(action_state_->polygon_name, "SlowdownOnAllSources");

  // Stop Collision Monitor node
  cm_->stop();
}

// =============================================================================
// Steering Field Angle Limiter Tests
// =============================================================================
// These tests verify the steering field angle limiter functionality where:
// - For each (speed, steering_angle) pair, there are 3 polygon types:
//   stop (emergency), slowdown (warning), and limit fields
// - Fields form a matrix with speed and steering angle as axes
// - Neighbor fields are adjacent in the matrix (next higher/lower speed or angle)
// - Low speed threshold is 0.3 m/s
// =============================================================================

TEST_F(Tester, testSteeringFieldWarningFullCheckNeighbor)
{
  // Test: When warning field is full, check neighbor steering field and slow down
  // Setup: Create velocity polygons for different steering angles
  // - Center (0 deg): stop=1.0m, slowdown=2.0m, limit=3.0m
  // - Left neighbor (+30 deg): stop=1.0m, slowdown=2.0m, limit=3.0m
  // - Right neighbor (-30 deg): stop=1.0m, slowdown=2.0m, limit=3.0m
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3);

  // Create velocity polygons for center steering (theta: -0.2 to 0.2 rad)
  addPolygon("SteeringCenter", VELOCITY_POLYGON, 1.0, "stop");
  addPolygonVelocitySubPolygon("SteeringCenter", "Forward", 0.0, 1.0, -0.2, 0.2, 1.0);
  setPolygonVelocityVectors("SteeringCenter", {"Forward"});

  // Create velocity polygon for left steering (theta: 0.2 to 0.7 rad)
  addPolygon("SteeringLeft", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringLeft", "ForwardLeft", 0.0, 1.0, 0.2, 0.7, 2.0);
  setPolygonVelocityVectors("SteeringLeft", {"ForwardLeft"});

  // Create velocity polygon for right steering (theta: -0.7 to -0.2 rad)
  addPolygon("SteeringRight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringRight", "ForwardRight", 0.0, 1.0, -0.7, -0.2, 2.0);
  setPolygonVelocityVectors("SteeringRight", {"ForwardRight"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"SteeringCenter", "SteeringLeft", "SteeringRight"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Scenario: Obstacle in center warning field, but left neighbor field is clear
  // Expected: System should check left neighbor and allow steering left with slowdown
  publishPointCloud(1.5, curr_time);  // Obstacle at 1.5m (in slowdown zone)
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Command forward with slight left steering
  publishCmdVel(0.5, 0.0, 0.3);  // theta=0.3 rad (in left steering range)
  ASSERT_TRUE(waitCmdVel(500ms));

  // Should slow down due to warning field, but not stop
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldProgressiveChecking)
{
  // Test: Different steering angles result in different polygon checks
  // Field1 has larger polygon than Field0, so steering left avoids obstacle
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3);

  // Single velocity polygon with multiple sub-polygons for different angles
  addPolygon("SteeringFields", VELOCITY_POLYGON, 1.0, "stop");
  // Center field: small polygon (1.0m), obstacle at 0.8m will trigger stop
  addPolygonVelocitySubPolygon("SteeringFields", "Center", 0.0, 1.0, -0.2, 0.2, 1.0);
  // Left field: larger polygon (2.0m), but obstacle at 0.8m still inside
  addPolygonVelocitySubPolygon("SteeringFields", "Left", 0.0, 1.0, 0.2, 0.6, 2.0);
  setPolygonVelocityVectors("SteeringFields", {"Center", "Left"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"SteeringFields"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at 0.8m - inside center (1.0m) and left (2.0m) polygons
  publishPointCloud(0.8, curr_time);
  ASSERT_TRUE(waitData(std::hypot(0.8, 0.01), 500ms, curr_time));

  // Forward (theta=0) - center field - should stop
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);

  // Steering left (theta=0.4) - left field - still stops (obstacle inside 2.0m)
  publishCmdVel(0.5, 0.0, 0.4);
  ASSERT_TRUE(waitCmdVel(500ms));
  // Both fields have obstacles, so robot should stop
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldLowSpeedBehavior)
{
  // Test: At low speed (< 0.3 m/s), allow entering neighbor steering field
  // At low speeds, the system can be more aggressive with steering changes
  // and risk E-Stop if necessary
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3);

  // Low speed field (0 to 0.3 m/s)
  addPolygon("LowSpeedCenter", VELOCITY_POLYGON, 1.0, "stop");
  addPolygonVelocitySubPolygon("LowSpeedCenter", "Slow", 0.0, 0.3, -0.5, 0.5, 0.5);
  setPolygonVelocityVectors("LowSpeedCenter", {"Slow"});

  // Low speed left field - smaller stop zone for low speed
  addPolygon("LowSpeedLeft", VELOCITY_POLYGON, 1.0, "stop");
  addPolygonVelocitySubPolygon("LowSpeedLeft", "SlowLeft", 0.0, 0.3, 0.5, 1.0, 0.5);
  setPolygonVelocityVectors("LowSpeedLeft", {"SlowLeft"});

  // High speed field (0.3 to 1.0 m/s)
  addPolygon("HighSpeedCenter", VELOCITY_POLYGON, 1.0, "stop");
  addPolygonVelocitySubPolygon("HighSpeedCenter", "Fast", 0.3, 1.0, -0.5, 0.5, 1.5);
  setPolygonVelocityVectors("HighSpeedCenter", {"Fast"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"LowSpeedCenter", "LowSpeedLeft", "HighSpeedCenter"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at medium distance
  publishPointCloud(1.0, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.0, 0.01), 500ms, curr_time));

  // At low speed (0.2 m/s), should be able to move since obstacle is outside small polygon
  publishCmdVel(0.2, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.2, EPSILON);

  // At high speed (0.5 m/s), should be stopped since obstacle is inside larger polygon
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldWarnFreeProgressiveSteering)
{
  // Test: When steering into a clear field, velocity passes through
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3);

  // Single velocity polygon covering left steering angles
  addPolygon("LeftField", VELOCITY_POLYGON, 1.0, "limit");
  addPolygonVelocitySubPolygon("LeftField", "Left", 0.0, 1.0, 0.2, 0.8, 3.0);
  setPolygonVelocityVectors("LeftField", {"Left"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"LeftField"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle far away (outside 3.0m polygon)
  publishPointCloud(4.0, curr_time);
  ASSERT_TRUE(waitData(std::hypot(4.0, 0.01), 500ms, curr_time));

  // Request left steering (theta=0.5) - in LeftField range, obstacle outside
  publishCmdVel(0.5, 0.0, 0.5);
  ASSERT_TRUE(waitCmdVel(500ms));

  // Field is clear, velocity should pass through
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.5, EPSILON);

  cm_->stop();

  // TODO: add test for changing steering field to higher limit than current field
  // -> limit steering to current limit if warn in next heigher neighbor
}

TEST_F(Tester, testSteeringFieldHighSpeedKeepsCurrentLogic)
{
  // Test: For very high speeds (> 1.0 m/s), use old/normal collision monitor logic
  // Steering field limiter only applies between 0.3 and 1.0 m/s
  setCommonParameters();
  // Enable limiter with thresholds: 0.3 (low) to 1.0 (high)
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // Velocity polygon covering very high speeds (> 1.0 m/s)
  addPolygon("VeryHighSpeedPolygon", VELOCITY_POLYGON, 1.0, "stop");
  addPolygonVelocitySubPolygon("VeryHighSpeedPolygon", "VeryFast", 1.0, 2.0, -1.0, 1.0, 2.0);
  setPolygonVelocityVectors("VeryHighSpeedPolygon", {"VeryFast"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"VeryHighSpeedPolygon"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle inside polygon
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Very high speed (1.5 m/s > 1.0 threshold) - steering field limiter doesn't apply
  // Standard collision monitor behavior: obstacle inside polygon triggers STOP
  publishCmdVel(1.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);

  // Obstacle outside polygon
  publishPointCloud(2.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(2.5, 0.01), 500ms, curr_time));

  // Very high speed with clear field - should pass through normally
  publishCmdVel(1.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 1.5, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldMatrixNeighborCheck)
{
  // Test: Single velocity polygon with sub-polygons for different speed/angle combos
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3);

  // Create a velocity polygon with sub-polygons for different speed ranges
  addPolygon("SpeedFields", VELOCITY_POLYGON, 1.0, "stop");
  // Low speed center: small polygon (0.8m)
  addPolygonVelocitySubPolygon("SpeedFields", "LowCenter", 0.0, 0.3, -0.5, 0.5, 0.8);
  // High speed center: larger polygon (1.5m)
  addPolygonVelocitySubPolygon("SpeedFields", "HighCenter", 0.3, 1.0, -0.5, 0.5, 1.5);
  setPolygonVelocityVectors("SpeedFields", {"LowCenter", "HighCenter"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"SpeedFields"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at 1.2m - inside high speed polygon (1.5m) but outside low speed (0.8m)
  publishPointCloud(1.2, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.2, 0.01), 500ms, curr_time));

  // Low speed (0.2 m/s) - obstacle outside 0.8m polygon, should pass
  publishCmdVel(0.2, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.2, EPSILON);

  // High speed (0.5 m/s) - obstacle inside 1.5m polygon, should stop
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, STOP);
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.0, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldLimitToStraight)
{
  // Test: When target field is blocked and no clear neighbor in target direction,
  // limit steering to 0 rad/s (straight) and slow down - don't steer opposite
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // Create velocity polygons for 3 steering ranges:
  // Straight field (theta: -0.2 to 0.2) - will be CLEAR
  // Little left field (theta: 0.2 to 0.4) - will be BLOCKED
  // Far left field (theta: 0.4 to 0.8) - will be BLOCKED (target direction)

  addPolygon("SteeringStraight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringStraight", "Straight", 0.0, 1.0, -0.2, 0.2, 3.0);
  setPolygonVelocityVectors("SteeringStraight", {"Straight"});

  addPolygon("SteeringLittleLeft", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringLittleLeft", "LittleLeft", 0.0, 1.0, 0.2, 0.4, 2.0);
  setPolygonVelocityVectors("SteeringLittleLeft", {"LittleLeft"});

  addPolygon("SteeringFarLeft", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringFarLeft", "FarLeft", 0.0, 1.0, 0.4, 0.8, 2.0);
  setPolygonVelocityVectors("SteeringFarLeft", {"FarLeft"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"SteeringStraight", "SteeringLittleLeft", "SteeringFarLeft"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at 1.5m - inside LittleLeft (2.0m) and FarLeft (2.0m) polygons,
  // but outside Straight (3.0m) polygon
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request far left steering (theta=0.6) at medium speed
  // Target field (FarLeft) is blocked, neighbor (LittleLeft) also blocked
  // No clear neighbor in target direction (positive)
  // Should limit steering to 0 rad/s (straight) and slow down
  publishCmdVel(0.5, 0.0, 0.6);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  // Should get SLOWDOWN action
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);

  // Steering should be limited to 0 rad/s (straight)
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  // Speed should be slowed down
  ASSERT_LT(cmd_vel_out_->linear.x, 0.5);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldNegativeDirection)
{
  // Test: Steering in negative direction (turning right)
  // Mirror of left steering tests to ensure symmetry
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // Create velocity polygons for right steering:
  // Straight field (theta: -0.2 to 0.2) - will be BLOCKED
  // Little right field (theta: -0.4 to -0.2) - will be CLEAR (neighbor)
  // Far right field (theta: -0.8 to -0.4) - target direction

  addPolygon("SteeringStraight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringStraight", "Straight", 0.0, 1.0, -0.2, 0.2, 2.0);
  setPolygonVelocityVectors("SteeringStraight", {"Straight"});

  addPolygon("SteeringLittleRight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringLittleRight", "LittleRight", 0.0, 1.0, -0.4, -0.2, 3.0);
  setPolygonVelocityVectors("SteeringLittleRight", {"LittleRight"});

  addPolygon("SteeringFarRight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringFarRight", "FarRight", 0.0, 1.0, -0.8, -0.4, 2.0);
  setPolygonVelocityVectors("SteeringFarRight", {"FarRight"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"SteeringStraight", "SteeringLittleRight", "SteeringFarRight"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at 1.5m - inside Straight (2.0m) and FarRight (2.0m),
  // but outside LittleRight (3.0m)
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request far right steering (theta=-0.6) at medium speed
  // Target field (FarRight) is blocked, but neighbor (LittleRight) is clear
  // Should limit steering to LittleRight boundary (-0.4) and slow down
  publishCmdVel(0.5, 0.0, -0.6);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  // Should get SLOWDOWN action
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);

  // Steering should be limited - either to neighbor boundary or to straight
  // Depending on neighbor search finding clear field
  ASSERT_GE(cmd_vel_out_->angular.z, -0.5);  // Should not go beyond -0.4 much
  ASSERT_LE(cmd_vel_out_->angular.z, 0.0);   // Should be at most straight (0)

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldNegativeLimitToStraight)
{
  // Test: Negative steering with all neighbors blocked -> limit to straight
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // All right-side fields blocked, only straight is clear
  addPolygon("SteeringStraight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringStraight", "Straight", 0.0, 1.0, -0.2, 0.2, 3.0);
  setPolygonVelocityVectors("SteeringStraight", {"Straight"});

  addPolygon("SteeringLittleRight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringLittleRight", "LittleRight", 0.0, 1.0, -0.4, -0.2, 2.0);
  setPolygonVelocityVectors("SteeringLittleRight", {"LittleRight"});

  addPolygon("SteeringFarRight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringFarRight", "FarRight", 0.0, 1.0, -0.8, -0.4, 2.0);
  setPolygonVelocityVectors("SteeringFarRight", {"FarRight"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"SteeringStraight", "SteeringLittleRight", "SteeringFarRight"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at 1.5m - blocks all right-side fields
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request far right steering - all neighbors blocked
  publishCmdVel(0.5, 0.0, -0.6);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  // Should limit to straight (0 rad/s)
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldClearNeighborAfterBlockedOnes)
{
  // Test: Find clear neighbor after passing through blocked fields
  // Setup: target blocked, first neighbor blocked, second neighbor clear
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // Far left (target) - blocked
  addPolygon("FarLeft", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("FarLeft", "FL", 0.0, 1.0, 0.6, 1.0, 2.0);
  setPolygonVelocityVectors("FarLeft", {"FL"});

  // Mid left (first neighbor) - blocked
  addPolygon("MidLeft", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("MidLeft", "ML", 0.0, 1.0, 0.3, 0.6, 2.0);
  setPolygonVelocityVectors("MidLeft", {"ML"});

  // Little left (second neighbor) - CLEAR
  addPolygon("LittleLeft", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("LittleLeft", "LL", 0.0, 1.0, 0.0, 0.3, 3.0);
  setPolygonVelocityVectors("LittleLeft", {"LL"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"FarLeft", "MidLeft", "LittleLeft"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle blocks FarLeft and MidLeft but not LittleLeft
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request far left steering (theta=0.8)
  publishCmdVel(0.5, 0.0, 0.8);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  // Should find LittleLeft as clear and limit to its max boundary (0.3)
  ASSERT_LE(cmd_vel_out_->angular.z, 0.35);  // Limited to around 0.3
  ASSERT_GE(cmd_vel_out_->angular.z, 0.0);   // Still positive

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldSpeedThresholdBoundary)
{
  // Test: Behavior exactly at speed threshold boundaries
  // At low speed (< 0.3), limiter is disabled, normal collision monitor applies
  // At medium speed (0.3-1.0), limiter is active
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // Polygon active for medium speeds (0.3-1.0) with slowdown
  addPolygon("MediumSpeed", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("MediumSpeed", "Med", 0.3, 1.0, -0.5, 0.5, 2.0);
  setPolygonVelocityVectors("MediumSpeed", {"Med"});

  // Low speed polygon - clear (large range so obstacle is outside)
  addPolygon("LowSpeed", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("LowSpeed", "Low", 0.0, 0.3, -0.5, 0.5, 3.0);
  setPolygonVelocityVectors("LowSpeed", {"Low"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"MediumSpeed", "LowSpeed"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at 1.5m - inside MediumSpeed polygon (2.0m) but outside LowSpeed (3.0m)
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // At exactly low speed threshold (0.3 m/s) - limiter should apply
  // Obstacle inside MediumSpeed polygon triggers slowdown
  publishCmdVel(0.3, 0.0, 0.0);  // Straight steering to avoid limiter interference
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  // Speed should be reduced
  ASSERT_LT(cmd_vel_out_->linear.x, 0.3);

  // Clear obstacle for next test
  publishPointCloud(10.0, curr_time);
  ASSERT_TRUE(waitData(std::hypot(10.0, 0.01), 500ms, curr_time));

  // Just below low threshold (0.29 m/s) - limiter disabled
  // With obstacle far away, should pass through regardless
  publishCmdVel(0.29, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  // Should pass through with no slowdown
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.29, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldAllFieldsBlocked)
{
  // Test: All fields including straight are blocked
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // All fields have same small range - all blocked
  addPolygon("Straight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Straight", "S", 0.0, 1.0, -0.2, 0.2, 2.0);
  setPolygonVelocityVectors("Straight", {"S"});

  addPolygon("Left", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Left", "L", 0.0, 1.0, 0.2, 0.6, 2.0);
  setPolygonVelocityVectors("Left", {"L"});

  addPolygon("Right", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Right", "R", 0.0, 1.0, -0.6, -0.2, 2.0);
  setPolygonVelocityVectors("Right", {"R"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"Straight", "Left", "Right"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle blocks all fields
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request left steering - all blocked, should limit to straight and slow
  publishCmdVel(0.5, 0.0, 0.4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  // Should limit to straight even though straight is also blocked
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  ASSERT_LT(cmd_vel_out_->linear.x, 0.5);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldTargetClearPassThrough)
{
  // Test: Target field is clear - velocity passes through unchanged
  // Note: steering limiter only affects steering angle, normal collision
  // monitor processing still applies to speed based on matched polygon
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  addPolygon("Straight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Straight", "S", 0.0, 1.0, -0.2, 0.2, 2.0);
  setPolygonVelocityVectors("Straight", {"S"});

  addPolygon("Left", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Left", "L", 0.0, 1.0, 0.2, 0.6, 3.0);
  setPolygonVelocityVectors("Left", {"L"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"Straight", "Left"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle far away - outside both polygons
  publishPointCloud(4.0, curr_time);
  ASSERT_TRUE(waitData(std::hypot(4.0, 0.01), 500ms, curr_time));

  // Request left steering - target (Left) is clear, obstacle outside
  publishCmdVel(0.5, 0.0, 0.4);
  ASSERT_TRUE(waitCmdVel(500ms));

  // Target is clear with obstacle outside, should pass through
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.4, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldLimitActionType)
{
  // Test: Steering field limiter with LIMIT action type polygon
  // Verify that LIMIT polygons also work with the steering field limiter
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  addPolygon("Straight", VELOCITY_POLYGON, 1.0, "limit");
  addPolygonVelocitySubPolygon("Straight", "S", 0.0, 1.0, -0.2, 0.2, 2.0);
  setPolygonVelocityVectors("Straight", {"S"});

  addPolygon("Left", VELOCITY_POLYGON, 1.0, "limit");
  addPolygonVelocitySubPolygon("Left", "L", 0.0, 1.0, 0.2, 0.6, 3.0);
  setPolygonVelocityVectors("Left", {"L"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"Straight", "Left"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle far away - outside both polygons
  publishPointCloud(4.0, curr_time);
  ASSERT_TRUE(waitData(std::hypot(4.0, 0.01), 500ms, curr_time));

  // Request left steering with obstacle far away
  publishCmdVel(0.5, 0.0, 0.4);
  ASSERT_TRUE(waitCmdVel(500ms));

  // All fields clear, should pass through
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.4, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldLimiterDisabled)
{
  // Test: When steering_field_limiter_enabled is false, normal behavior applies
  // Normal slowdown scales all velocities (linear and angular) by slowdown ratio
  setCommonParameters();
  enableSteeringFieldLimiter(false, 0.3, 1.0);  // DISABLED

  addPolygon("Straight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Straight", "S", 0.0, 1.0, -0.2, 0.2, 2.0);
  setPolygonVelocityVectors("Straight", {"S"});

  addPolygon("Left", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Left", "L", 0.0, 1.0, 0.2, 0.6, 2.0);
  setPolygonVelocityVectors("Left", {"L"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"Straight", "Left"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle blocks both fields
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request left steering - with limiter disabled, normal slowdown applies
  publishCmdVel(0.5, 0.0, 0.4);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  // Should apply slowdown normally (no steering field special logic)
  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  // With limiter disabled, angular velocity is NOT forced to 0
  // Normal slowdown scales angular too: 0.4 * 0.7 = 0.28
  ASSERT_GT(cmd_vel_out_->angular.z, 0.0);  // Should not be forced to 0
  ASSERT_LT(cmd_vel_out_->angular.z, 0.5);  // Should be slowed

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldMultipleSubPolygonsSingleVelocityPolygon)
{
  // Test: Single VelocityPolygon with multiple sub-polygons for angles
  // Verify steering field limiter works with complex polygon configurations
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  // Single velocity polygon with 5 sub-polygons for different angles
  addPolygon("SteeringMatrix", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("SteeringMatrix", "FarRight", 0.0, 1.0, -0.8, -0.4, 2.0);
  addPolygonVelocitySubPolygon("SteeringMatrix", "LittleRight", 0.0, 1.0, -0.4, -0.1, 2.0);
  addPolygonVelocitySubPolygon("SteeringMatrix", "Straight", 0.0, 1.0, -0.1, 0.1, 3.0);
  addPolygonVelocitySubPolygon("SteeringMatrix", "LittleLeft", 0.0, 1.0, 0.1, 0.4, 2.0);
  addPolygonVelocitySubPolygon("SteeringMatrix", "FarLeft", 0.0, 1.0, 0.4, 0.8, 2.0);
  setPolygonVelocityVectors(
    "SteeringMatrix",
    {"FarRight", "LittleRight", "Straight", "LittleLeft", "FarLeft"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"SteeringMatrix"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle at 1.5m - blocks all except Straight (3.0m)
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request far left (0.6) - blocked, neighbors blocked, limit to straight
  publishCmdVel(0.5, 0.0, 0.6);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);
  // Speed is slowed down
  ASSERT_LT(cmd_vel_out_->linear.x, 0.5);

  // Update obstacle position far away to clear all fields
  publishPointCloud(4.0, curr_time);
  ASSERT_TRUE(waitData(std::hypot(4.0, 0.01), 500ms, curr_time));

  // Now request straight (0.0) - all fields clear
  publishCmdVel(0.5, 0.0, 0.0);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_NEAR(cmd_vel_out_->linear.x, 0.5, EPSILON);
  ASSERT_NEAR(cmd_vel_out_->angular.z, 0.0, EPSILON);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldSmallAngleRequest)
{
  // Test: Small steering angle just into blocked neighbor
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  addPolygon("Straight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Straight", "S", 0.0, 1.0, -0.2, 0.2, 3.0);
  setPolygonVelocityVectors("Straight", {"S"});

  addPolygon("Left", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Left", "L", 0.0, 1.0, 0.2, 0.6, 2.0);
  setPolygonVelocityVectors("Left", {"L"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"Straight", "Left"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle blocks left field only
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request small left angle (0.25) - just inside blocked Left field
  // Neighbor search should find Straight as clear
  publishCmdVel(0.5, 0.0, 0.25);
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  // Should limit to Straight field boundary (0.2)
  ASSERT_LE(cmd_vel_out_->angular.z, 0.25);
  ASSERT_GE(cmd_vel_out_->angular.z, 0.0);

  cm_->stop();
}

TEST_F(Tester, testSteeringFieldYVelocityPreserved)
{
  // Test: Y velocity (lateral) is also scaled with slowdown
  setCommonParameters();
  enableSteeringFieldLimiter(true, 0.3, 1.0);

  addPolygon("Straight", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Straight", "S", 0.0, 1.0, -0.3, 0.3, 3.0);
  setPolygonVelocityVectors("Straight", {"S"});

  addPolygon("Left", VELOCITY_POLYGON, 1.0, "slowdown");
  addPolygonVelocitySubPolygon("Left", "L", 0.0, 1.0, 0.3, 0.8, 2.0);
  setPolygonVelocityVectors("Left", {"L"});

  addSource(POINTCLOUD_NAME, POINTCLOUD);
  setVectors({"Straight", "Left"}, {POINTCLOUD_NAME});

  rclcpp::Time curr_time = cm_->now();
  cm_->start();
  sendTransforms(curr_time);

  // Obstacle blocks left
  publishPointCloud(1.5, curr_time);
  ASSERT_TRUE(waitData(std::hypot(1.5, 0.01), 500ms, curr_time));

  // Request with Y velocity
  publishCmdVel(0.5, 0.2, 0.5);  // Include Y velocity
  ASSERT_TRUE(waitCmdVel(500ms));
  ASSERT_TRUE(waitActionState(500ms));

  ASSERT_EQ(action_state_->action_type, SLOWDOWN);
  // Both X and Y should be scaled
  ASSERT_LT(cmd_vel_out_->linear.x, 0.5);
  ASSERT_LT(cmd_vel_out_->linear.y, 0.2);
  // Steering limited
  ASSERT_LE(cmd_vel_out_->angular.z, 0.35);

  cm_->stop();
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
