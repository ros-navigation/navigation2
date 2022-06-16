// Copyright (c) 2022 Samsung Research Russia
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

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <exception>
#include <utility>
#include <functional>

#include "geometry_msgs/msg/point.hpp"

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_monitor", "", false, options),
  odom_frame_id_(""), footprint_sub_(nullptr), process_active_(false),
  action_type_prev_(DO_NOTHING)
{
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
}

nav2_util::CallbackReturn
CollisionMonitor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string odom_topic;
  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;
  std::string footprint_topic;

  // Obtaining ROS parameters
  if (!getParameters(odom_topic, cmd_vel_in_topic, cmd_vel_out_topic, footprint_topic)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Setting subscribers and publishers
  rclcpp::QoS odom_qos = rclcpp::SystemDefaultsQoS();  // set to default
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, odom_qos,
    std::bind(&CollisionMonitor::odomCallback, this, std::placeholders::_1));

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic, 1,
    std::bind(&CollisionMonitor::cmdVelInCallback, this, std::placeholders::_1));
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_out_topic, 1);

  if (approach_ && footprint_topic.length() > 0) {
    footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
      shared_from_this(), footprint_topic, *tf_buffer_,
      base_frame_id_, tf2::durationToSec(transform_tolerance_));
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();

  // Activating STOP/SLOWDOWN polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Activating footprint APPROACH polygon
  if (approach_) {
    footprint_->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  action_type_prev_ = DO_NOTHING;

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating footprint polygon
  if (approach_) {
    footprint_->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  footprint_sub_.reset();
  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();
  odom_sub_.reset();
  odom_frame_id_.clear();

  polygons_.clear();
  footprint_.reset();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void CollisionMonitor::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // Obtaining odom_frame_id_ from first message and setting it as global frame for data sources
  if (odom_frame_id_.length() == 0) {
    odom_frame_id_ = msg->header.frame_id;
  }
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  process({msg->linear.x, msg->linear.y, msg->angular.z});
}

void CollisionMonitor::publishVelocity(const Velocity & vel) const
{
  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg =
    std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = vel.x;
  cmd_vel_out_msg->linear.y = vel.y;
  cmd_vel_out_msg->angular.z = vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

bool CollisionMonitor::getParameters(
  std::string & odom_topic,
  std::string & cmd_vel_in_topic,
  std::string & cmd_vel_out_topic,
  std::string & footprint_topic)
{
  try {
    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(
      node, "odom_topic", rclcpp::ParameterValue("odom"));
    odom_topic = get_parameter("odom_topic").as_string();

    nav2_util::declare_parameter_if_not_declared(
      node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_raw"));
    cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
    nav2_util::declare_parameter_if_not_declared(
      node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
    cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();

    nav2_util::declare_parameter_if_not_declared(
      node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    nav2_util::declare_parameter_if_not_declared(
      node, "transform_tolerance", rclcpp::ParameterValue(0.1));
    transform_tolerance_ =
      tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    nav2_util::declare_parameter_if_not_declared(
      node, "data_timeout", rclcpp::ParameterValue(2.0));
    data_timeout_ =
      tf2::durationFromSec(get_parameter("data_timeout").as_double());

    nav2_util::declare_parameter_if_not_declared(
      node, "approach", rclcpp::ParameterValue(false));
    approach_ = get_parameter("approach").as_bool();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  if (!configurePolygons()) {
    return false;
  }

  if (approach_) {
    if (!configureFootprint(footprint_topic)) {
      return false;
    }
  }

  if (!configureSources()) {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons()
{
  try {
    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::ParameterValue(std::vector<std::string>()));
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        polygons_.push_back(std::make_shared<Polygon>(node, polygon_name));
      } else if (polygon_type == "circle") {
        polygons_.push_back(std::make_shared<Circle>(node, polygon_name));
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown polygon type: %s",
          polygon_name.c_str() ,polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }

      // Added polygon should be STOP or SLOWDOWN action type
      if (polygons_.back()->getActionType() != STOP &&
        polygons_.back()->getActionType() != SLOWDOWN)
      {
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Should be \"stop\" or \"slowdown\" action type",
          polygon_name.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureFootprint(std::string & footprint_topic)
{
  try {
    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(
      node, "footprint_name", rclcpp::ParameterValue("FootprintApproach"));
    std::string footprint_name = get_parameter("footprint_name").as_string();

    nav2_util::declare_parameter_if_not_declared(
      node, "time_before_collision", rclcpp::ParameterValue(2.0));
    time_before_collision_ = get_parameter("time_before_collision").as_double();
    nav2_util::declare_parameter_if_not_declared(
      node, "simulation_time_step", rclcpp::ParameterValue(0.02));
    simulation_time_step_ = get_parameter("simulation_time_step").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, footprint_name + ".type", rclcpp::PARAMETER_STRING);
    const std::string footprint_type = get_parameter(footprint_name + ".type").as_string();

    if (footprint_type == "polygon") {
      // Create a new footprint subscription only for polygon footprint type
      nav2_util::declare_parameter_if_not_declared(
        node, "footprint_topic", rclcpp::ParameterValue("footprint"));
      footprint_topic = get_parameter("footprint_topic").as_string();

      footprint_ = std::make_shared<Polygon>(node, footprint_name);
    } else if (footprint_type == "circle") {
      // If circle footprint type was chosen, it will be supposed to be static in a time:
      // no footprint subscription is required
      footprint_topic = "";

      footprint_ = std::make_shared<Circle>(node, footprint_name);
    } else {  // Error if something else
      RCLCPP_ERROR(
        get_logger(),
        "[%s]: Unknown footprint type: %s",
        footprint_name.c_str() ,footprint_type.c_str());
      return false;
    }

    // Configure footprint polygon
    if (!footprint_->configure()) {
      return false;
    }

    // Footprint should be APPROACH action type
    if (footprint_->getActionType() != APPROACH) {
      RCLCPP_ERROR(
        get_logger(),
        "[%s]: Should be \"approach\" action type",
        footprint_name.c_str());
      return false;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources()
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(node, source_name);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(node, source_name);

        p->configure();

        sources_.push_back(p);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown source type: %s",
          source_name.c_str(), source_type.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::process(const Velocity & cmd_vel_in)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  // Warn if odometry was not received yet and do nothing
  if (odom_frame_id_.length() == 0) {
    RCLCPP_WARN(get_logger(), "Odometry is not received yet");
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_) {
    source->getData(
      base_frame_id_, curr_time, odom_frame_id_, transform_tolerance_,
      data_timeout_, tf_buffer_, collision_points);
  }

  // By default - there is no action
  Action robot_action{DO_NOTHING, cmd_vel_in};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  // Process STOP and SLOWDOWN polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    // Process selected polygon
    if (processStopSlowdown(polygon, collision_points, cmd_vel_in, robot_action)) {
      action_polygon = polygon;
    }
  }

  // Process APPROACH footprint
  if (approach_ && robot_action.action_type != STOP) {
    if (processApproach(collision_points, cmd_vel_in, robot_action)) {
      action_polygon = footprint_;
    }
  }

  if (robot_action.action_type != action_type_prev_) {
    // Report changed robot behavior
    printAction(robot_action, action_polygon);
    action_type_prev_ = robot_action.action_type;
  }

  // Publish requred robot velocity
  publishVelocity(robot_action.req_vel);

  // Publish polygons for better visualization
  publishPolygons();
}

bool CollisionMonitor::processStopSlowdown(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (footprint_sub_ != nullptr) {
    // Get robot footprint from footprint subscriber
    std::vector<geometry_msgs::msg::Point> footprint_vec;
    std_msgs::msg::Header footprint_header;
    footprint_sub_->getFootprintInRobotFrame(footprint_vec, footprint_header);

    // Update footprint polygon points with latest received message
    footprint_->setPolygon(footprint_vec);
  }

  // Obtain time before a collision
  const double collision_time = getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0)
  {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / time_before_collision_;
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

double CollisionMonitor::getCollisionTime(
  const std::vector<Point> & collision_points,
  const Velocity & velocity) const
{
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0.0, 0.0, 0.0};
  Velocity vel = velocity;

  // Array of points transformed to the frame concerned with pose on each simulation step
  std::vector<Point> points_transformed;

  // Robot movement simulation
  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {
    // Shift the robot pose towards to the vel during simulation_time_step_ time interval
    // NOTE: vel is changing during the simulation
    projectState(simulation_time_step_, pose, vel);
    // Transform collision_points to the frame concerned with current robot pose
    points_transformed = collision_points;
    transformPoints(pose, points_transformed);
    // If the collision occurred on this stage, return the actual time before a collision
    // as if robot was moved with given velocity
    if (footprint_->getPointsInside(points_transformed) > footprint_->getMaxPoints()) {
      return time;
    }
  }

  // There is no collision
  return -1.0;
}

void CollisionMonitor::printAction(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to stop due to %s polygon",
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to slowdown for %f percents due to %s polygon",
      action_polygon->getSlowdownRatio() * 100,
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to approach for %f seconds away from collision",
      time_before_collision_);
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(
      get_logger(),
      "Robot to continue normal operation");
  }
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->publish(base_frame_id_);
  }

  if (approach_) {
    footprint_->publish(base_frame_id_);
  }
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
