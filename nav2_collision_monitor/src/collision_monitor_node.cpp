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
#include <cmath>
#include <limits>
#include <utility>

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

CollisionMonitorNode::CollisionMonitorNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_monitor_node", "", false, options),
  velocity_({0.0, 0.0, 0.0}), velocity_valid_(false), cmd_vel_in_({0.0, 0.0, 0.0}),
  ra_prev_({DO_NOTHING, {0.0, 0.0, 0.0}}), release_operation_(false)
{
  RCLCPP_INFO(get_logger(), "Creating CollisionMonitorNode");
}

CollisionMonitorNode::~CollisionMonitorNode()
{
  RCLCPP_INFO(get_logger(), "Destroying CollisionMonitorNode");
  polygons_.clear();
  sources_.clear();
}

nav2_util::CallbackReturn
CollisionMonitorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

  // Obtaining ROS parameters
  if (!getParameters()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Setting subscribers and publishers
  rclcpp::QoS odom_qos = rclcpp::SystemDefaultsQoS();  // set to default
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, odom_qos,
    std::bind(&CollisionMonitorNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic_, 1,
    std::bind(&CollisionMonitorNode::cmdVelInCallback, this, std::placeholders::_1));
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_out_topic_, 1);

  if (polygon_topic_.length()) {
    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      polygon_topic_, polygon_qos);
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitorNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publishers
  polygon_pub_->on_activate();
  cmd_vel_out_pub_->on_activate();

  // Activating main worker
  setWorkerActive(true);

  // Creating bond connection
  createBond();

  // Polygons publishing thread
  polygon_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&CollisionMonitorNode::publishPolygons, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitorNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  setWorkerActive(false);

  // Reset opeartion states to their defaults after worker deactivating
  release_operation_ = false;
  ra_prev_ = {DO_NOTHING, {0.0, 0.0, 0.0}};

  // Deactivating lifecycle publishers
  polygon_pub_->on_deactivate();
  cmd_vel_out_pub_->on_deactivate();

  // Deactivating polygons publishing timer
  polygon_pub_timer_->cancel();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  tf_listener_.reset();
  tf_buffer_.reset();

  odom_sub_.reset();
  cmd_vel_in_sub_.reset();

  polygon_pub_.reset();
  cmd_vel_out_pub_.reset();

  polygon_pub_timer_.reset();

  polygons_.clear();
  sources_.clear();

  resetVelocity();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitorNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void CollisionMonitorNode::publishPolygons()
{
  if (!polygon_topic_.length()) {
    // Topic is not specified: there is no need to publish
    return;
  }

  // Fill PolygonStamped struct
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();
  poly_s->header.stamp = this->now();
  poly_s->header.frame_id = base_frame_id_;
  poly_s->polygon = polygons_pub_;

  // Publish polygon
  polygon_pub_->publish(std::move(poly_s));
}

void CollisionMonitorNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<mutex_t> lock(velocity_mutex_);

  double dt;
  try {
    dt = (this->now() - msg->header.stamp).seconds();
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "Can not get difference between collision monitor and odom time: %s",
      ex.what());
    dt = 0.0;
  }

  if (std::fabs(dt) <= max_time_shift_) {
    velocity_.x = msg->twist.twist.linear.x;
    velocity_.y = msg->twist.twist.linear.y;
    velocity_.tw = msg->twist.twist.angular.z;
    velocity_valid_ = true;
  } else {
    // Warn and ignore the odom data if it and Collision Monitor node have different time stamps
    RCLCPP_WARN(
      get_logger(),
      "Odom and collision monitor node timestamps differ on %f seconds. Ignoring odom.",
      dt);
    velocity_valid_ = false;
  }
}

Velocity CollisionMonitorNode::getVelocity()
{
  std::lock_guard<mutex_t> lock(velocity_mutex_);
  return velocity_;
}

bool CollisionMonitorNode::velocityValid()
{
  std::lock_guard<mutex_t> lock(velocity_mutex_);
  return velocity_valid_;
}

void CollisionMonitorNode::resetVelocity()
{
  std::lock_guard<mutex_t> lock(velocity_mutex_);
  velocity_ = {0.0, 0.0, 0.0};
  velocity_valid_ = false;
}

void CollisionMonitorNode::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  std::lock_guard<mutex_t> lock(cmd_vel_in_mutex_);
  cmd_vel_in_ = {msg->linear.x, msg->linear.y, msg->angular.z};

  workerMain();
}

Velocity CollisionMonitorNode::getCmdVelIn()
{
  std::lock_guard<mutex_t> lock(cmd_vel_in_mutex_);
  return cmd_vel_in_;
}

void CollisionMonitorNode::publishVelocity(const Velocity & vel)
{
  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg =
    std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = vel.x;
  cmd_vel_out_msg->linear.y = vel.y;
  cmd_vel_out_msg->linear.z = 0.0;
  cmd_vel_out_msg->angular.x = 0.0;
  cmd_vel_out_msg->angular.y = 0.0;
  cmd_vel_out_msg->angular.z = vel.tw;

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

bool CollisionMonitorNode::getParameters()
{
  try {
    auto node = shared_from_this();

    // Initializing polygon topic with empty string.
    // If it won't be re-writed, polygons won't be published.
    nav2_util::declare_parameter_if_not_declared(
      node, "polygon_topic", rclcpp::ParameterValue(""));
    polygon_topic_ = get_parameter("polygon_topic").as_string();
    nav2_util::declare_parameter_if_not_declared(
      node, "odom_topic", rclcpp::ParameterValue("odom"));
    odom_topic_ = get_parameter("odom_topic").as_string();
    nav2_util::declare_parameter_if_not_declared(
      node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_raw"));
    cmd_vel_in_topic_ = get_parameter("cmd_vel_in_topic").as_string();
    nav2_util::declare_parameter_if_not_declared(
      node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
    cmd_vel_out_topic_ = get_parameter("cmd_vel_out_topic").as_string();

    nav2_util::declare_parameter_if_not_declared(
      node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    nav2_util::declare_parameter_if_not_declared(
      node, "transform_tolerance", rclcpp::ParameterValue(0.1));
    double transform_tolerance = get_parameter("transform_tolerance").as_double();
    nav2_util::declare_parameter_if_not_declared(
      node, "max_time_shift", rclcpp::ParameterValue(2.0));
    max_time_shift_ = get_parameter("max_time_shift").as_double();
    nav2_util::declare_parameter_if_not_declared(
      node, "simulation_time_step", rclcpp::ParameterValue(0.02));
    double simulation_time_step = get_parameter("simulation_time_step").as_double();

    nav2_util::declare_parameter_if_not_declared(
      node, "release_step", rclcpp::ParameterValue(0.002));
    release_step_ = get_parameter("release_step").as_double();

    // Leave these parameters to be not initialized:
    // the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::PARAMETER_STRING_ARRAY);
    nav2_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);

    // Working with polygons parameters
    polygon_names_ = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names_) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        std::shared_ptr<Polygon> p = std::make_shared<Polygon>(
          node, polygon_name, simulation_time_step);

        if (!p->getParameters()) {
          return false;
        }

        polygons_.push_back(p);
      } else {  // polygon_type == "circle"
        std::shared_ptr<Circle> c = std::make_shared<Circle>(
          node, polygon_name, simulation_time_step);

        if (!c->getParameters()) {
          return false;
        }

        polygons_.push_back(c);
      }
    }

    // Working with data sources parameters
    source_names_ = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names_) {
      nav2_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, tf_buffer_, source_name, base_frame_id_,
          transform_tolerance, max_time_shift_);

        if (!s->init()) {
          return false;
        }

        sources_.push_back(s);
      } else {  // source_type == "pointcloud"
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, tf_buffer_, source_name, base_frame_id_,
          transform_tolerance, max_time_shift_);

        if (!p->init()) {
          return false;
        }

        sources_.push_back(p);
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  // Store polygons coordinates for a future re-use when publishing
  std::vector<Point> poly;
  for (std::shared_ptr<PolygonBase> polygon : polygons_) {
    polygon->getPolygon(poly);
    for (Point p : poly) {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      p_s.z = 0.0;
      polygons_pub_.points.push_back(p_s);
    }
  }

  return true;
}

void CollisionMonitorNode::workerMain()
{
  // Do nothing if main worker in non-active state
  // or if velocity is not received yet
  if (!getWorkerActive() || !velocityValid()) {
    return;
  }

  // Get latest valid velocity for further usage
  Velocity velocity = getVelocity();
  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;
  rclcpp::Time curr_time = this->now();

  // Fill collision_points array from different data sources
  for (std::shared_ptr<SourceBase> source : sources_) {
    source->getData(collision_points, curr_time, velocity);
  }

  // ToDo: Add nearest points bounds estimation by using all polygons and current velocity
  // and then cut-off of extra points out of calculated bounds

  // By default - there is no action
  Action ra{DO_NOTHING,{0.0, 0.0, 0.0}};
  // Used for APPROACH model to define "nearest" obstacle
  double min_collision_time = std::numeric_limits<double>::max();
  // Current robot operation velocity: maximum possible one
  Velocity max_vel = getCmdVelIn();

  for (std::shared_ptr<PolygonBase> polygon : polygons_) {
    const ActionType at = polygon->getActionType();
    if (ra.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    if (at == STOP || at == SLOWDOWN) {
      int points_inside = 0;

      for (Point point : collision_points) {
        if (polygon->isPointInside(point)) {
          points_inside++;
        }
      }

      if (points_inside >= polygon->getStopPoints()) {
        if (at == STOP) {
          // For stopping model zero safe velocity should be set
          // without any checks (with highest priority)
          ra.action_type = STOP;
          ra.req_vel.x = 0.0;
          ra.req_vel.y = 0.0;
          ra.req_vel.tw = 0.0;
        } else {  // at == SLOWDOWN
          const Velocity safe_vel = max_vel * polygon->getSlowdown();
          // Check that currently calculated velocity is safer than
          // chosen for previous shapes one
          if (ra.action_type == DO_NOTHING || safe_vel < ra.req_vel) {
            ra.action_type = SLOWDOWN;
            ra.req_vel = safe_vel;
          }
        }
      }
    } else if (at == APPROACH) {
      double collision_time;
      for (Point point : collision_points) {
        polygon->getCollision(point, velocity, collision_time);
        if (collision_time > 0.0 &&
          collision_time <= polygon->getTimeBeforeCollision() &&
          collision_time < min_collision_time)
        {
          const Velocity safe_vel = polygon->getSafeVelocity(velocity, collision_time);
          // Check that currently calculated velocity is safer than
          // chosen for previous shapes one
          if (ra.action_type == DO_NOTHING || safe_vel < ra.req_vel) {
            ra.action_type = APPROACH;
            ra.req_vel = safe_vel;
            min_collision_time = collision_time;
          }
        }
      }
    }
  }

  if (ra.action_type == DO_NOTHING) {
    if (ra_prev_.action_type == APPROACH) {
      release_operation_ = true;
    }

    if (!release_operation_) {
      // Continue normal robot operation
      ra.req_vel = max_vel;
    } else {
      // Release robot from approach: gradually increase robot speed in order to avoid twitching
      const Velocity release_vel = ra_prev_.req_vel+=release_step_;
      if (release_vel < max_vel) {
        ra.req_vel = release_vel;
      } else {
        ra.req_vel = max_vel;
        // Robot already reached its normal speed: releasing its to usual operation
        release_operation_ = false;
      }
    }
  }

  if (ra != ra_prev_) {
    // Report action type
    if (ra.action_type == STOP) {
      RCLCPP_INFO(get_logger(), "Robot to stop");
    } else if (ra.action_type == SLOWDOWN) {
      RCLCPP_INFO(
        get_logger(),
        "Robot to slowdown for (Vx, Vy, Tw) = (%f, %f, %f)",
        ra.req_vel.x, ra.req_vel.y, ra.req_vel.tw);
    } else if (ra.action_type == APPROACH) {
      RCLCPP_INFO(
        get_logger(),
        "Robot to approach for (Vx, Vy, Tw) = (%f, %f, %f)",
        ra.req_vel.x, ra.req_vel.y, ra.req_vel.tw);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Robot to release for (Vx, Vy, Tw) = (%f, %f, %f)",
        ra.req_vel.x, ra.req_vel.y, ra.req_vel.tw);
    }

    // Publish required robot velocity
    publishVelocity(ra.req_vel);

    // Set prev robot action to avoid duplicating publications
    ra_prev_ = ra;
  }
}

void CollisionMonitorNode::setWorkerActive(const bool worker_active)
{
  std::lock_guard<mutex_t> lock(worker_active_mutex_);
  worker_active_ = worker_active;
}

bool CollisionMonitorNode::getWorkerActive()
{
  std::lock_guard<mutex_t> lock(worker_active_mutex_);
  return worker_active_;
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitorNode)
