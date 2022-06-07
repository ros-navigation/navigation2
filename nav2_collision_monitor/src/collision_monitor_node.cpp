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
#include <limits>
#include <utility>

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_monitor", "", false, options),
  first_odom_(false), velocity_({0.0, 0.0, 0.0}), velocity_received_(false),
  velocity_stamp_{0, 0, get_clock()->get_clock_type()}, cmd_vel_in_({0.0, 0.0, 0.0}),
  process_active_(false), robot_action_prev_({DO_NOTHING, {0.0, 0.0, 0.0}}),
  release_operation_(false), prev_time_{0, 0, get_clock()->get_clock_type()}
{
  RCLCPP_INFO(get_logger(), "Creating CollisionMonitor");
}

CollisionMonitor::~CollisionMonitor()
{
  RCLCPP_INFO(get_logger(), "Destroying CollisionMonitor");
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

  // Obtaining ROS parameters
  if (!getParameters(odom_topic, cmd_vel_in_topic, cmd_vel_out_topic)) {
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

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publishers
  cmd_vel_out_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<PolygonBase> polygon : polygons_) {
    polygon->activate();
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

  // Reset opeartion states to their defaults after worker deactivating
  release_operation_ = false;
  robot_action_prev_ = {DO_NOTHING, {0.0, 0.0, 0.0}};

  // Deactivating polygons
  for (std::shared_ptr<PolygonBase> polygon : polygons_) {
    polygon->deactivate();
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

  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();
  odom_sub_.reset();
  first_odom_ = false;
  velocity_received_ = false;

  polygons_.clear();
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
  // Obtaining odom_frame_id_ from first message and setting it as fixed frame for data sources
  if (!first_odom_) {
    for (std::shared_ptr<SourceBase> source : sources_) {
      source->setFixedFrameId(msg->header.frame_id);
    }
    first_odom_ = true;
  }

  velocity_.x = msg->twist.twist.linear.x;
  velocity_.y = msg->twist.twist.linear.y;
  velocity_.tw = msg->twist.twist.angular.z;
  velocity_stamp_ = msg->header.stamp;
  velocity_received_ = true;
}

bool CollisionMonitor::velocityValid(const rclcpp::Time & curr_time)
{
  // Check whether velocity is not received yet
  if (!velocity_received_) {
    return false;
  }

  // Check that latest received odom timestamp is earlier
  // than current time more than data_timeout_ interval
  const rclcpp::Duration dt = curr_time - velocity_stamp_;
  if (dt > data_timeout_) {
    RCLCPP_WARN(
      get_logger(),
      "Latest odom and current collision monitor node timestamps differ on %f seconds.",
      dt.seconds());
    return false;
  }

  return true;
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  cmd_vel_in_ = {msg->linear.x, msg->linear.y, msg->angular.z};
  processMain();
}

void CollisionMonitor::publishVelocity(const Velocity & vel)
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
  std::string & cmd_vel_out_topic)
{
  try {
    auto node = shared_from_this();

    // Initializing polygon topic with empty string.
    // If it won't be re-writed, polygons won't be published.
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
    std::string base_frame_id = get_parameter("base_frame_id").as_string();
    nav2_util::declare_parameter_if_not_declared(
      node, "transform_tolerance", rclcpp::ParameterValue(0.1));
    tf2::Duration transform_tolerance =
      tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    nav2_util::declare_parameter_if_not_declared(
      node, "data_timeout", rclcpp::ParameterValue(2.0));
    data_timeout_ =
      tf2::durationFromSec(get_parameter("data_timeout").as_double());
    nav2_util::declare_parameter_if_not_declared(
      node, "simulation_time_step", rclcpp::ParameterValue(0.02));
    double simulation_time_step = get_parameter("simulation_time_step").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, "release_acceleration", rclcpp::ParameterValue(0.1));
    release_acceleration_ = get_parameter("release_acceleration").as_double();

    // Leave these parameters to be not initialized:
    // the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::PARAMETER_STRING_ARRAY);
    nav2_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);

    // Working with polygons parameters
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        std::shared_ptr<Polygon> p = std::make_shared<Polygon>(
          node, polygon_name, base_frame_id, simulation_time_step);

        if (!p->configure()) {
          return false;
        }

        polygons_.push_back(p);
      } else if (polygon_type == "circle") {
        std::shared_ptr<Circle> c = std::make_shared<Circle>(
          node, polygon_name, base_frame_id, simulation_time_step);

        if (!c->configure()) {
          return false;
        }

        polygons_.push_back(c);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown polygon type: %s",
          polygon_name.c_str() ,polygon_type.c_str());
        return false;
      }
    }

    // Working with data sources parameters
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, tf_buffer_, source_name, base_frame_id,
          transform_tolerance, data_timeout_);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, tf_buffer_, source_name, base_frame_id,
          transform_tolerance, data_timeout_);

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

void CollisionMonitor::processMain()
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  // or if velocity is not received yet
  if (!process_active_ || !velocityValid(curr_time)) {
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<SourceBase> source : sources_) {
    source->getData(curr_time, collision_points);
  }

  // By default - there is no action
  Action robot_action{DO_NOTHING, {0.0, 0.0, 0.0}};
  // Used for APPROACH model to define "nearest" obstacle
  double min_collision_time = std::numeric_limits<double>::max();

  for (std::shared_ptr<PolygonBase> polygon : polygons_) {
    const ActionType at = polygon->getActionType();
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    // Process selected polygon
    if (at == STOP || at == SLOWDOWN) {
      processStopSlowdown(polygon, collision_points, cmd_vel_in_, robot_action);
    } else if (at == APPROACH) {
      processApproach(polygon, collision_points, velocity_, min_collision_time, robot_action);
    }
  }

  // In APPROACH model due to sensors noise calculated speed of robot might be instantly changed
  // causing robot to twitch. To avoid this, when releasing robot after APPROACH,
  // its speed should be increased gradually smoothing the twitches over time until robot will
  // reach its desired normal opration.
  if (robot_action.action_type == DO_NOTHING) {
    if (robot_action_prev_.action_type == APPROACH) {
      // Trigger gradually velocity increase
      release_operation_ = true;
    }

    if (!release_operation_) {
      // Continue normal robot operation
      robot_action.req_vel = cmd_vel_in_;
    } else {
      // Release robot from approach: accelerate the robot (without changing speed instantly)
      // in order to avoid twitching
      releaseOperation(curr_time, cmd_vel_in_, robot_action);
    }
  }

  if (robot_action != robot_action_prev_) {
    // Report robot action type and changed velocity
    printAction(robot_action);

    // Publish required robot velocity
    publishVelocity(robot_action.req_vel);

    // Set previous robot action to avoid duplicating publications
    robot_action_prev_ = robot_action;
    // Set previous robot time to calculate speed change when release normal operation
    prev_time_ = curr_time;
  }

  publishPolygons();
}

void CollisionMonitor::processStopSlowdown(
  const std::shared_ptr<PolygonBase> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action)
{
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for stopping model
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
    } else {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (robot_action.action_type == DO_NOTHING || safe_vel < robot_action.req_vel) {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
      }
    }
  }
}

void CollisionMonitor::processApproach(
    const std::shared_ptr<PolygonBase> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    double & min_collision_time,
    Action & robot_action)
{
  const double collision_time = polygon->getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0 &&
    collision_time <= polygon->getTimeBeforeCollision() &&
    collision_time < min_collision_time)
  {
    const Velocity safe_vel = polygon->getSafeVelocity(velocity, collision_time);
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (robot_action.action_type == DO_NOTHING || safe_vel < robot_action.req_vel) {
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      min_collision_time = collision_time;
    }
  }
}

void CollisionMonitor::releaseOperation(
  const rclcpp::Time & curr_time, const Velocity & desired_vel, Action & robot_action)
{
  Velocity release_vel = robot_action_prev_.req_vel;
  // Previous robot speed
  const double prev_vel_module =
    std::sqrt(release_vel.x * release_vel.x + release_vel.y * release_vel.y);
  // Speed increase step: acceleration during (curr_time - prev_time_) interval
  const double vel_change = release_acceleration_ * (curr_time - prev_time_).seconds();
  // Maximum (desired) robot speed
  const double desired_vel_module =
    std::sqrt(desired_vel.x * desired_vel.x + desired_vel.y * desired_vel.y);

  // Calculate speed ratio as previous speed increased on acceleration
  // during (curr_time - prev_time_) time interval towards desired velocity
  const double change_ratio = (prev_vel_module + vel_change) / desired_vel_module;
  release_vel.x = desired_vel.x * change_ratio;
  release_vel.y = desired_vel.y * change_ratio;
  release_vel.tw = desired_vel.tw * change_ratio;

  // Check that approximately increased velocity does not exceed the desired speed
  if (release_vel < desired_vel) {
    robot_action.req_vel = release_vel;
  } else {
    robot_action.req_vel = desired_vel;
    // Robot already reached its desired speed: continuing normal operation
    release_operation_ = false;
  }
}

void CollisionMonitor::printAction(const Action & robot_action)
{
  if (robot_action.action_type == STOP) {
    RCLCPP_INFO(get_logger(), "Robot to stop");
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to slowdown for (Vx, Vy, Tw) = (%f, %f, %f)",
      robot_action.req_vel.x, robot_action.req_vel.y, robot_action.req_vel.tw);
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to approach for (Vx, Vy, Tw) = (%f, %f, %f)",
      robot_action.req_vel.x, robot_action.req_vel.y, robot_action.req_vel.tw);
  } else {  // robot_action.action_type == DO_NOTHING
    if (release_operation_) {
      RCLCPP_INFO(
        get_logger(),
        "Robot to release for (Vx, Vy, Tw) = (%f, %f, %f)",
        robot_action.req_vel.x, robot_action.req_vel.y, robot_action.req_vel.tw);
    }
  }
}

void CollisionMonitor::publishPolygons()
{
  for (std::shared_ptr<PolygonBase> polygon : polygons_) {
    polygon->publish();
  }
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
