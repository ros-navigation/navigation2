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

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <algorithm>
#include <exception>
#include <utility>
#include <functional>

#include "tf2_ros/create_timer_ros.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"

namespace nav2_collision_monitor
{

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("collision_monitor", options),
  process_active_(false), robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}, ""},
  stop_stamp_{0, 0, get_clock()->get_clock_type()}, stop_pub_timeout_(1.0, 0.0)
{
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
}

nav2::CallbackReturn
CollisionMonitor::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;
  std::string state_topic;

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic, state_topic)) {
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  cmd_vel_in_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    shared_from_this(),
    cmd_vel_in_topic,
    std::bind(&CollisionMonitor::cmdVelInCallbackUnstamped, this, std::placeholders::_1),
    std::bind(&CollisionMonitor::cmdVelInCallbackStamped, this, std::placeholders::_1));

  auto node = shared_from_this();
  cmd_vel_out_pub_ = std::make_unique<nav2_util::TwistPublisher>(node, cmd_vel_out_topic);

  if (!state_topic.empty()) {
    state_pub_ = this->create_publisher<nav2_msgs::msg::CollisionMonitorState>(
      state_topic);
  }

  collision_points_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/collision_points_marker");

  nav2::declare_parameter_if_not_declared(
    node, "use_realtime_priority", rclcpp::ParameterValue(false));
  bool use_realtime_priority = false;
  node->get_parameter("use_realtime_priority", use_realtime_priority);
  if (use_realtime_priority) {
    try {
      nav2::setSoftRealTimePriority();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();
  if (state_pub_) {
    state_pub_->on_activate();
  }
  collision_points_marker_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}, ""};

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();
  if (state_pub_) {
    state_pub_->on_deactivate();
  }
  collision_points_marker_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();
  state_pub_.reset();
  collision_points_marker_pub_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2::CallbackReturn::SUCCESS;
}

void CollisionMonitor::cmdVelInCallbackStamped(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(msg->twist)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  process({msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z}, msg->header);
}

void CollisionMonitor::cmdVelInCallbackUnstamped(geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto twist_stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped->twist = *msg;
  cmdVelInCallbackStamped(twist_stamped);
}

void CollisionMonitor::publishVelocity(
  const Action & robot_action, const std_msgs::msg::Header & header)
{
  if (robot_action.req_vel.isZero()) {
    if (!robot_action_prev_.req_vel.isZero()) {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = this->now();
    } else if (this->now() - stop_stamp_ > stop_pub_timeout_) {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel.
      return;
    }
  }

  auto cmd_vel_out_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel_out_msg->header = header;
  cmd_vel_out_msg->twist.linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->twist.linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->twist.angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

bool CollisionMonitor::getParameters(
  std::string & cmd_vel_in_topic,
  std::string & cmd_vel_out_topic,
  std::string & state_topic)
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();

  nav2::declare_parameter_if_not_declared(
    node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_smoothed"));
  cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
  nav2::declare_parameter_if_not_declared(
    node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();
  nav2::declare_parameter_if_not_declared(
    node, "state_topic", rclcpp::ParameterValue(""));
  state_topic = get_parameter("state_topic").as_string();

  nav2::declare_parameter_if_not_declared(
    node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_parameter("base_frame_id").as_string();
  nav2::declare_parameter_if_not_declared(
    node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_parameter("odom_frame_id").as_string();
  nav2::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance =
    tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
  nav2::declare_parameter_if_not_declared(
    node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout =
    rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());
  nav2::declare_parameter_if_not_declared(
    node, "base_shift_correction", rclcpp::ParameterValue(true));
  const bool base_shift_correction =
    get_parameter("base_shift_correction").as_bool();

  nav2::declare_parameter_if_not_declared(
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  // Steering field angle limiter parameters
  nav2::declare_parameter_if_not_declared(
    node, "steering_field_limiter_enabled", rclcpp::ParameterValue(false));
  steering_field_limiter_enabled_ =
    get_parameter("steering_field_limiter_enabled").as_bool();

  nav2::declare_parameter_if_not_declared(
    node, "low_speed_threshold", rclcpp::ParameterValue(0.3));
  low_speed_threshold_ =
    get_parameter("low_speed_threshold").as_double();

  nav2::declare_parameter_if_not_declared(
    node, "high_speed_threshold", rclcpp::ParameterValue(1.0));
  high_speed_threshold_ =
    get_parameter("high_speed_threshold").as_double();

  if (
    !configureSources(
      base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction))
  {
    return false;
  }

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        polygons_.push_back(
          std::make_shared<Polygon>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "circle") {
        polygons_.push_back(
          std::make_shared<Circle>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "velocity_polygon") {
        polygons_.push_back(
          std::make_shared<VelocityPolygon>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown polygon type: %s",
          polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(
  const std::string & base_frame_id,
  const std::string & odom_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        p->configure();

        sources_.push_back(p);
      } else if (source_type == "range") {
        std::shared_ptr<Range> r = std::make_shared<Range>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        r->configure();

        sources_.push_back(r);
      } else if (source_type == "polygon") {
        std::shared_ptr<PolygonSource> ps = std::make_shared<PolygonSource>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);
        ps->configure();

        sources_.push_back(ps);
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

void CollisionMonitor::process(const Velocity & cmd_vel_in, const std_msgs::msg::Header & header)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::unordered_map<std::string, std::vector<Point>> sources_collision_points_map;

  // By default - there is no action
  Action robot_action{DO_NOTHING, cmd_vel_in, ""};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  // Fill collision points array from different data sources
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  for (std::shared_ptr<Source> source : sources_) {
    auto iter = sources_collision_points_map.insert(
      {source->getSourceName(), std::vector<Point>()});

    if (source->getEnabled()) {
      if (!source->getData(curr_time, iter.first->second) &&
        source->getSourceTimeout().seconds() != 0.0)
      {
        action_polygon = nullptr;
        robot_action.polygon_name = "invalid source";
        robot_action.action_type = STOP;
        robot_action.req_vel.x = 0.0;
        robot_action.req_vel.y = 0.0;
        robot_action.req_vel.tw = 0.0;
        break;
      }
    }

    if (collision_points_marker_pub_->get_subscription_count() > 0) {
      // visualize collision points with markers
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = get_parameter("base_frame_id").as_string();
      marker.header.stamp = rclcpp::Time(0, 0);
      marker.ns = "collision_points_" + source->getSourceName();
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      marker.lifetime = rclcpp::Duration(0, 0);
      marker.frame_locked = true;

      for (const auto & point : iter.first->second) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        marker.points.push_back(p);
      }
      marker_array->markers.push_back(marker);
    }
  }

  if (collision_points_marker_pub_->get_subscription_count() > 0) {
    collision_points_marker_pub_->publish(std::move(marker_array));
  }

  // Process steering field angle limiter for medium speed maneuvers
  // This may modify robot_action if steering needs to be limited
  processSteeringFieldLimiter(cmd_vel_in, sources_collision_points_map, robot_action,
    action_polygon);

  // Store the velocity for polygon processing - this is either cmd_vel_in (if steering field
  // limiter is disabled) or a modified velocity (if steering was limited)
  // Use this consistently for all polygon processing to avoid cascading modifications
  const Velocity velocity_for_processing = robot_action.req_vel;

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (!polygon->getEnabled()) {
      continue;
    }
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    // Update polygon coordinates
    polygon->updatePolygon(velocity_for_processing);

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN || at == LIMIT) {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdownLimit(
          polygon, sources_collision_points_map, velocity_for_processing, robot_action))
      {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, sources_collision_points_map, velocity_for_processing,
          robot_action))
      {
        action_polygon = polygon;
      }
    }
  }

  if (robot_action.polygon_name != robot_action_prev_.polygon_name) {
    // Report changed robot behavior
    notifyActionState(robot_action, action_polygon);
  }

  // Publish required robot velocity
  publishVelocity(robot_action, header);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

bool CollisionMonitor::processStopSlowdownLimit(
  const std::shared_ptr<Polygon> polygon,
  const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (!polygon->isShapeSet()) {
    return false;
  }

  if (polygon->getPointsInside(sources_collision_points_map) >= polygon->getMinPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else if (polygon->getActionType() == SLOWDOWN) {
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    } else {  // Limit
      // Compute linear velocity
      const double linear_vel = std::hypot(velocity.x, velocity.y);  // absolute
      Velocity safe_vel;
      double ratio = 1.0;

      // Calculate the most restrictive ratio to preserve curvature
      if (linear_vel != 0.0) {
        ratio = std::min(ratio, polygon->getLinearLimit() / linear_vel);
      }
      if (velocity.tw != 0.0) {
        ratio = std::min(ratio, polygon->getAngularLimit() / std::abs(velocity.tw));
      }
      ratio = std::clamp(ratio, 0.0, 1.0);
      // Apply the same ratio to all components to preserve curvature
      safe_vel = velocity * ratio;
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = LIMIT;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(
  const std::shared_ptr<Polygon> polygon,
  const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (!polygon->isShapeSet()) {
    return false;
  }

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(sources_collision_points_map, velocity);
  if (collision_time >= 0.0) {
    // If collision will occur, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::notifyActionState(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP) {
    if (robot_action.polygon_name == "invalid source") {
      RCLCPP_WARN(
        get_logger(),
        "Robot to stop due to invalid source."
        " Either due to data not published yet, or to lack of new data received within the"
        " sensor timeout, or if impossible to transform data to base frame");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Robot to stop due to %s polygon",
        action_polygon->getName().c_str());
    }
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to slowdown for %f percents due to %s polygon",
      action_polygon->getSlowdownRatio() * 100,
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == LIMIT) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to limit speed due to %s polygon",
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to approach for %f seconds away from collision",
      action_polygon->getTimeBeforeCollision());
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(
      get_logger(),
      "Robot to continue normal operation");
  }

  if (state_pub_) {
    std::unique_ptr<nav2_msgs::msg::CollisionMonitorState> state_msg =
      std::make_unique<nav2_msgs::msg::CollisionMonitorState>();
    state_msg->polygon_name = robot_action.polygon_name;
    state_msg->action_type = robot_action.action_type;

    state_pub_->publish(std::move(state_msg));
  }
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (polygon->getEnabled()) {
      polygon->publish();
    }
  }
}

bool CollisionMonitor::processSteeringFieldLimiter(
  const Velocity & cmd_vel_in,
  const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map,
  Action & robot_action,
  std::shared_ptr<Polygon> & action_polygon)
{
  if (!steering_field_limiter_enabled_) {
    return false;
  }

  const double linear_vel = cmd_vel_in.x;
  const double angular_vel = cmd_vel_in.tw;

  // For low speeds (< low_speed_threshold_), don't apply steering field limiter
  // Let the robot enter neighbor field and risk E-stop if necessary
  if (std::abs(linear_vel) < low_speed_threshold_) {
    return false;
  }

  // For high speeds (> high_speed_threshold_), don't apply steering field limiter
  // Use the old/normal collision monitor logic
  if (std::abs(linear_vel) > high_speed_threshold_) {
    return false;
  }

  // For medium speeds (low_speed_threshold_ <= speed <= high_speed_threshold_),
  // apply steering field limiter logic:
  // - Check if warning field is full
  // - If full, check neighbor steering field and slow down
  // - Progressively limit steering to neighbor fields until desired angle is reached

  // Find polygons matching the requested velocity
  auto matching_polygons = findPolygonsForVelocity(linear_vel, angular_vel);

  if (matching_polygons.empty()) {
    // No matching polygons, try to find neighbor fields progressively
    // Determine direction to search for neighbors based on angular velocity sign
    int direction = (angular_vel >= 0) ? 1 : -1;

    auto neighbor_polygons = findNeighborSteeringFields(angular_vel, linear_vel, direction);

    for (const auto & neighbor : neighbor_polygons) {
      // Update neighbor polygon with a velocity in its range to set its shape
      double neighbor_theta_min, neighbor_theta_max;
      if (!getPolygonAngularRange(neighbor, neighbor_theta_min, neighbor_theta_max)) {
        neighbor->updatePolygon(cmd_vel_in);
        continue;
      }

      // Use the middle of the neighbor's theta range
      Velocity neighbor_vel = cmd_vel_in;
      neighbor_vel.tw = (neighbor_theta_min + neighbor_theta_max) / 2.0;
      neighbor->updatePolygon(neighbor_vel);

      if (isFieldClear(neighbor, sources_collision_points_map)) {
        // Neighbor field is clear, limit steering to this field's range
        // Only cap the steering - never force more steering than requested
        double limited_angular = angular_vel;
        if (direction > 0) {
          // Searching toward positive angles, cap at neighbor's max
          limited_angular = std::min(angular_vel, neighbor_theta_max);
        } else {
          // Searching toward negative angles, cap at neighbor's min
          limited_angular = std::max(angular_vel, neighbor_theta_min);
        }

        robot_action.req_vel.tw = limited_angular;
        robot_action.action_type = LIMIT;
        robot_action.polygon_name = neighbor->getName();
        action_polygon = neighbor;

        RCLCPP_DEBUG(
          get_logger(),
          "Steering field limiter: limited angular from %.3f to %.3f via %s",
          angular_vel, limited_angular, neighbor->getName().c_str());
        return true;
      }
      // Neighbor field has obstacles, continue to next neighbor
    }
  } else {
    // We have matching polygons, check if warning field is full
    for (const auto & polygon : matching_polygons) {
      // Skip STOP polygons - they should be handled by normal polygon processing
      // Steering field limiter only applies to SLOWDOWN and LIMIT polygons
      if (polygon->getActionType() == STOP) {
        continue;
      }

      polygon->updatePolygon(cmd_vel_in);

      if (!isFieldClear(polygon, sources_collision_points_map)) {
        // Warning field is full, check neighbor steering fields and slow down
        int direction = (angular_vel >= 0) ? 1 : -1;
        auto neighbors = findNeighborSteeringFields(angular_vel, linear_vel, direction);

        bool found_clear_neighbor = false;
        for (const auto & neighbor : neighbors) {
          // Update neighbor polygon with a velocity in its range to set its shape
          double theta_min, theta_max;
          if (getPolygonAngularRange(neighbor, theta_min, theta_max)) {
            // Use the middle of the neighbor's theta range
            Velocity neighbor_vel = cmd_vel_in;
            neighbor_vel.tw = (theta_min + theta_max) / 2.0;
            neighbor->updatePolygon(neighbor_vel);
          } else {
            neighbor->updatePolygon(cmd_vel_in);
          }

          if (isFieldClear(neighbor, sources_collision_points_map)) {
            // Neighbor field is clear, limit steering to that field
            // Only cap the steering - never force more steering than requested
            double limit_theta_min, limit_theta_max;
            if (getPolygonAngularRange(neighbor, limit_theta_min, limit_theta_max)) {
              double limited_angular = angular_vel;
              if (direction > 0) {
                // Searching toward positive angles, cap at neighbor's max
                limited_angular = std::min(angular_vel, limit_theta_max);
              } else {
                // Searching toward negative angles, cap at neighbor's min
                limited_angular = std::max(angular_vel, limit_theta_min);
              }

              robot_action.req_vel.tw = limited_angular;
              robot_action.action_type = SLOWDOWN;
              robot_action.polygon_name = neighbor->getName();
              action_polygon = neighbor;

              RCLCPP_INFO(
                get_logger(),
                "Steering field limiter: warning field full, steering to %s (angular: %.3f -> %.3f)",
                neighbor->getName().c_str(), angular_vel, limited_angular);
              found_clear_neighbor = true;
              return true;
            }
          }
        }

        // If no clear neighbor found in target direction, limit to straight (0 rad/s)
        // and slow down - don't steer into the opposite direction
        if (!found_clear_neighbor) {
          robot_action.req_vel.tw = 0.0;
          robot_action.action_type = SLOWDOWN;
          robot_action.polygon_name = polygon->getName();
          robot_action.req_vel.x = cmd_vel_in.x * polygon->getSlowdownRatio();
          robot_action.req_vel.y = cmd_vel_in.y * polygon->getSlowdownRatio();
          action_polygon = polygon;

          RCLCPP_INFO(
            get_logger(),
            "Steering field limiter: target blocked, no clear neighbor, "
            "limiting to straight (0 rad/s), slowing down");
          return true;
        }
      }
    }
  }

  return false;
}

std::vector<std::shared_ptr<Polygon>> CollisionMonitor::findPolygonsForVelocity(
  double linear_vel, double angular_vel) const
{
  std::vector<std::shared_ptr<Polygon>> result;

  for (const auto & polygon : polygons_) {
    auto velocity_polygon = std::dynamic_pointer_cast<VelocityPolygon>(polygon);
    if (velocity_polygon && velocity_polygon->isVelocityInRange(linear_vel, angular_vel)) {
      result.push_back(polygon);
    }
  }

  return result;
}

std::vector<std::shared_ptr<Polygon>> CollisionMonitor::findNeighborSteeringFields(
  double current_angular, double linear_vel, int direction) const
{
  std::vector<std::pair<double, std::shared_ptr<Polygon>>> candidates;

  for (const auto & polygon : polygons_) {
    auto velocity_polygon = std::dynamic_pointer_cast<VelocityPolygon>(polygon);
    if (!velocity_polygon) {
      continue;
    }

    double linear_min, linear_max;
    if (!velocity_polygon->getLinearRange(linear_min, linear_max)) {
      continue;
    }

    // Check if linear velocity is in range
    if (linear_vel < linear_min || linear_vel > linear_max) {
      continue;
    }

    double theta_min, theta_max;
    if (!velocity_polygon->getAngularRange(theta_min, theta_max)) {
      continue;
    }

    // Check if this is a neighbor in the specified direction
    if (direction > 0) {
      // Looking for polygons with higher angles
      if (theta_min > current_angular) {
        double distance = theta_min - current_angular;
        candidates.push_back({distance, polygon});
      }
    } else {
      // Looking for polygons with lower angles
      if (theta_max < current_angular) {
        double distance = current_angular - theta_max;
        candidates.push_back({distance, polygon});
      }
    }
  }

  // Sort by distance (closest first)
  std::sort(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});

  std::vector<std::shared_ptr<Polygon>> result;
  for (const auto & candidate : candidates) {
    result.push_back(candidate.second);
  }

  return result;
}

bool CollisionMonitor::isFieldClear(
  const std::shared_ptr<Polygon> polygon,
  const std::unordered_map<std::string, std::vector<Point>> & sources_collision_points_map) const
{
  if (!polygon->isShapeSet()) {
    return true;  // If shape not set, consider it clear
  }

  return polygon->getPointsInside(sources_collision_points_map) < polygon->getMinPoints();
}

bool CollisionMonitor::getPolygonAngularRange(
  const std::shared_ptr<Polygon> polygon,
  double & theta_min, double & theta_max) const
{
  auto velocity_polygon = std::dynamic_pointer_cast<VelocityPolygon>(polygon);
  if (!velocity_polygon) {
    return false;
  }

  return velocity_polygon->getAngularRange(theta_min, theta_max);
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
