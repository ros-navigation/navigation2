// Copyright (c) 2024, Open Navigation LLC
// Copyright (c) 2026, Dexory (Tony Najjar)
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

#include "nav2_loopback_sim/loopback_simulator.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <chrono>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace nav2_loopback_sim
{

LoopbackSimulator::LoopbackSimulator(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("loopback_simulator", options),
  curr_cmd_vel_time_(this->now())
{
}

nav2::CallbackReturn
LoopbackSimulator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Declare and get parameters
  update_dur_ = declare_or_get_parameter("update_duration", 0.01);
  base_frame_id_ = declare_or_get_parameter("base_frame_id", std::string("base_footprint"));
  map_frame_id_ = declare_or_get_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ = declare_or_get_parameter("odom_frame_id", std::string("odom"));
  scan_frame_id_ = declare_or_get_parameter("scan_frame_id", std::string("base_scan"));
  odom_publish_dur_ = declare_or_get_parameter("odom_publish_dur", update_dur_);
  scan_publish_dur_ = declare_or_get_parameter("scan_publish_dur", 0.1);
  publish_map_odom_tf_ = declare_or_get_parameter("publish_map_odom_tf", true);
  scan_range_min_ = declare_or_get_parameter("scan_range_min", 0.05);
  scan_range_max_ = declare_or_get_parameter("scan_range_max", 30.0);
  scan_angle_min_ = declare_or_get_parameter("scan_angle_min", -M_PI);
  scan_angle_max_ = declare_or_get_parameter("scan_angle_max", M_PI);
  scan_angle_increment_ = declare_or_get_parameter("scan_angle_increment", 0.0261);
  use_inf_ = declare_or_get_parameter("scan_use_inf", true);
  scan_noise_std_ = declare_or_get_parameter("scan_noise_std", 0.01);
  publish_scan_ = declare_or_get_parameter("publish_scan", true);
  publish_clock_ = declare_or_get_parameter("publish_clock", true);
  speed_factor_ = declare_or_get_parameter("speed_factor", 1.0);

  // Setup transforms
  t_map_to_odom_.header.frame_id = map_frame_id_;
  t_map_to_odom_.child_frame_id = odom_frame_id_;
  t_odom_to_base_link_.header.frame_id = odom_frame_id_;
  t_odom_to_base_link_.child_frame_id = base_frame_id_;

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Subscriptions
  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose",
    std::bind(&LoopbackSimulator::initialPoseCallback, this, _1));

  cmd_vel_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    shared_from_this(), "cmd_vel",
    std::bind(&LoopbackSimulator::cmdVelCallback, this, _1),
    std::bind(&LoopbackSimulator::cmdVelStampedCallback, this, _1));

  // Publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom");

  if (publish_scan_) {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", nav2::qos::SensorDataQoS());
  }

  if (publish_scan_) {
    map_client_ = create_client<nav_msgs::srv::GetMap>("/map_server/map");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  if (publish_clock_) {
    clock_publisher_ = std::make_unique<ClockPublisher>(
      weak_from_this(),
      speed_factor_);
  }

  param_validator_ = add_on_set_parameters_callback(
    std::bind(
      &LoopbackSimulator::validateParameterUpdatesCallback, this,
      std::placeholders::_1));
  param_updater_ = add_post_set_parameters_callback(
    std::bind(
      &LoopbackSimulator::updateParametersCallback, this,
      std::placeholders::_1));

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
LoopbackSimulator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activate lifecycle-managed subscriptions created in on_configure(); under
  // the create-on-activate wrapper they would otherwise stay dormant.
  if (initial_pose_sub_) {
    initial_pose_sub_->on_activate();
  }
  if (cmd_vel_sub_) {
    cmd_vel_sub_->on_activate();
  }

  odom_pub_->on_activate();
  if (scan_pub_) {
    scan_pub_->on_activate();
  }

  setup_timer_ = this->create_timer(
    100ms,
    std::bind(&LoopbackSimulator::setupTimerCallback, this));

  if (clock_publisher_) {
    clock_publisher_->start();
  }

  createBond();

  RCLCPP_INFO(get_logger(), "Loopback simulator activated");
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
LoopbackSimulator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (setup_timer_) {
    setup_timer_->cancel();
    setup_timer_.reset();
  }
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  if (odom_timer_) {
    odom_timer_->cancel();
    odom_timer_.reset();
  }
  if (scan_timer_) {
    scan_timer_->cancel();
    scan_timer_.reset();
  }

  if (clock_publisher_) {
    clock_publisher_->stop();
  }

  odom_pub_->on_deactivate();
  if (scan_pub_) {
    scan_pub_->on_deactivate();
  }
  if (initial_pose_sub_) {
    initial_pose_sub_->on_deactivate();
  }
  if (cmd_vel_sub_) {
    cmd_vel_sub_->on_deactivate();
  }

  has_initial_pose_ = false;
  curr_cmd_vel_.reset();

  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
LoopbackSimulator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  initial_pose_sub_.reset();
  cmd_vel_sub_.reset();
  odom_pub_.reset();
  scan_pub_.reset();
  map_client_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tf_broadcaster_.reset();
  clock_publisher_.reset();
  param_validator_.reset();
  param_updater_.reset();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
LoopbackSimulator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2::CallbackReturn::SUCCESS;
}

void LoopbackSimulator::getMap()
{
  if (!map_client_->wait_for_service(0s)) {
    return;
  }
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  map_client_->async_call(
    request,
    [this](typename rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
      auto response = future.get();
      if (response->map.info.width == 0 || response->map.info.height == 0 ||
      response->map.info.resolution <= 0.0)
      {
        RCLCPP_WARN(
          get_logger(),
          "Map server returned empty/invalid map (%dx%d, res=%.3f), will retry",
          response->map.info.width, response->map.info.height,
          response->map.info.resolution);
        return;
      }
      map_ = response->map;
      has_map_ = true;
      RCLCPP_INFO(get_logger(), "Laser scan will be populated using map data");
    });
}

void LoopbackSimulator::getBaseToLaserTf()
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      base_frame_id_, scan_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(transform.transform, tf_base_to_laser_);
    has_base_to_laser_ = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Transform lookup failed: %s", ex.what());
  }
}

void LoopbackSimulator::setupTimerCallback()
{
  t_odom_to_base_link_.header.stamp = this->now();
  tf_broadcaster_->sendTransform(t_odom_to_base_link_);
  if (publish_scan_ && !has_map_) {
    getMap();
  }
  if (publish_scan_ && !has_base_to_laser_) {
    getBaseToLaserTf();
  }
}

void LoopbackSimulator::cmdVelCallback(
  const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  RCLCPP_DEBUG(get_logger(), "Received cmd_vel");
  if (!has_initial_pose_) {
    return;
  }
  curr_cmd_vel_ = *msg;
  curr_cmd_vel_time_ = this->now();
}

void LoopbackSimulator::cmdVelStampedCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg)
{
  RCLCPP_DEBUG(get_logger(), "Received cmd_vel");
  if (!has_initial_pose_) {
    return;
  }
  curr_cmd_vel_ = msg->twist;
  curr_cmd_vel_time_ = rclcpp::Time(msg->header.stamp);
}

void LoopbackSimulator::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  RCLCPP_INFO(get_logger(), "Received initial pose!");

  if (!has_initial_pose_) {
    has_initial_pose_ = true;
    initial_pose_ = msg->pose.pose;

    // Initialize map->odom from input pose, odom->base_link starts as identity
    t_map_to_odom_.transform.translation.x = initial_pose_.position.x;
    t_map_to_odom_.transform.translation.y = initial_pose_.position.y;
    t_map_to_odom_.transform.rotation = initial_pose_.orientation;
    t_odom_to_base_link_.transform.translation = geometry_msgs::msg::Vector3();
    t_odom_to_base_link_.transform.rotation = geometry_msgs::msg::Quaternion();
    publishTransforms(t_map_to_odom_, t_odom_to_base_link_);

    // Cancel setup timer and start update/scan timers
    if (setup_timer_) {
      setup_timer_->cancel();
      setup_timer_.reset();
    }
    timer_ = this->create_timer(
      std::chrono::duration<double>(update_dur_),
      std::bind(&LoopbackSimulator::timerCallback, this));
    odom_timer_ = this->create_timer(
      std::chrono::duration<double>(odom_publish_dur_),
      std::bind(&LoopbackSimulator::odomTimerCallback, this));
    if (publish_scan_) {
      scan_timer_ = this->create_timer(
        std::chrono::duration<double>(scan_publish_dur_),
        std::bind(&LoopbackSimulator::publishLaserScan, this));
    }
    return;
  }

  initial_pose_ = msg->pose.pose;

  // Adjust map->odom based on new initial pose, keeping odom->base_link the same
  tf2::Transform tf_map_to_base;
  tf_map_to_base.setOrigin(
    tf2::Vector3(initial_pose_.position.x, initial_pose_.position.y, 0.0));
  tf_map_to_base.setRotation(
    tf2::Quaternion(
      initial_pose_.orientation.x, initial_pose_.orientation.y,
      initial_pose_.orientation.z, initial_pose_.orientation.w));

  tf2::Transform tf_odom_to_base;
  tf2::fromMsg(t_odom_to_base_link_.transform, tf_odom_to_base);

  tf2::Transform tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse();
  t_map_to_odom_.transform = tf2::toMsg(tf_map_to_odom);
}

void LoopbackSimulator::timerCallback()
{
  // If no data, just republish existing transforms without change
  auto one_sec = rclcpp::Duration::from_seconds(1.0);
  if (!curr_cmd_vel_.has_value() || (this->now() - curr_cmd_vel_time_) > one_sec) {
    publishTransforms(t_map_to_odom_, t_odom_to_base_link_);
    curr_cmd_vel_.reset();
    return;
  }

  // Update odom->base_link from cmd_vel
  double dx = curr_cmd_vel_->linear.x * update_dur_;
  double dy = curr_cmd_vel_->linear.y * update_dur_;
  double dth = curr_cmd_vel_->angular.z * update_dur_;

  tf2::Quaternion q(
    t_odom_to_base_link_.transform.rotation.x,
    t_odom_to_base_link_.transform.rotation.y,
    t_odom_to_base_link_.transform.rotation.z,
    t_odom_to_base_link_.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  t_odom_to_base_link_.transform.translation.x += dx * std::cos(yaw) - dy * std::sin(yaw);
  t_odom_to_base_link_.transform.translation.y += dx * std::sin(yaw) + dy * std::cos(yaw);
  t_odom_to_base_link_.transform.rotation =
    addYawToQuat(t_odom_to_base_link_.transform.rotation, dth);

  publishTransforms(t_map_to_odom_, t_odom_to_base_link_);
}

void LoopbackSimulator::odomTimerCallback()
{
  publishOdometry(t_odom_to_base_link_);
}

void LoopbackSimulator::publishLaserScan()
{
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header.stamp = this->now();
  scan_msg->header.frame_id = scan_frame_id_;
  scan_msg->angle_min = static_cast<float>(scan_angle_min_);
  scan_msg->angle_max = static_cast<float>(scan_angle_max_);
  scan_msg->angle_increment = static_cast<float>(scan_angle_increment_);
  scan_msg->time_increment = 0.0f;
  scan_msg->scan_time = static_cast<float>(scan_publish_dur_);
  scan_msg->range_min = static_cast<float>(scan_range_min_);
  scan_msg->range_max = static_cast<float>(scan_range_max_);

  int num_samples = static_cast<int>(
    (scan_angle_max_ - scan_angle_min_) / scan_angle_increment_);
  scan_msg->ranges.assign(num_samples, 0.0f);
  if (!has_map_) {
    getMap();
  }
  if (!has_base_to_laser_) {
    getBaseToLaserTf();
  }
  getLaserScan(num_samples, *scan_msg);
  scan_pub_->publish(std::move(scan_msg));
}

void LoopbackSimulator::publishTransforms(
  geometry_msgs::msg::TransformStamped & map_to_odom,
  geometry_msgs::msg::TransformStamped & odom_to_base_link)
{
  auto now = this->now();
  map_to_odom.header.stamp = now + rclcpp::Duration::from_seconds(update_dur_);
  odom_to_base_link.header.stamp = now;
  if (publish_map_odom_tf_) {
    tf_broadcaster_->sendTransform(map_to_odom);
  }
  tf_broadcaster_->sendTransform(odom_to_base_link);
}

void LoopbackSimulator::publishOdometry(
  const geometry_msgs::msg::TransformStamped & odom_to_base_link)
{
  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->header.stamp = this->now();
  odom->header.frame_id = odom_frame_id_;
  odom->child_frame_id = base_frame_id_;
  odom->pose.pose.position.x = odom_to_base_link.transform.translation.x;
  odom->pose.pose.position.y = odom_to_base_link.transform.translation.y;
  odom->pose.pose.orientation = odom_to_base_link.transform.rotation;
  if (curr_cmd_vel_.has_value()) {
    odom->twist.twist = curr_cmd_vel_.value();
  }
  odom_pub_->publish(std::move(odom));
}

geometry_msgs::msg::Quaternion LoopbackSimulator::addYawToQuat(
  const geometry_msgs::msg::Quaternion & quaternion, double yaw_to_add)
{
  tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, yaw_to_add);
  q = q * q_yaw;
  q.normalize();
  return tf2::toMsg(q);
}

std::tuple<double, double, double> LoopbackSimulator::getLaserPose()
{
  tf2::Transform tf_map_to_odom;
  tf2::fromMsg(t_map_to_odom_.transform, tf_map_to_odom);

  tf2::Transform tf_odom_to_base;
  tf2::fromMsg(t_odom_to_base_link_.transform, tf_odom_to_base);

  tf2::Transform tf_map_to_laser = tf_map_to_odom * tf_odom_to_base * tf_base_to_laser_;

  double x = tf_map_to_laser.getOrigin().x();
  double y = tf_map_to_laser.getOrigin().y();
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_map_to_laser.getRotation()).getRPY(roll, pitch, yaw);

  return {x, y, yaw};
}

void LoopbackSimulator::getLaserScan(
  int num_samples, sensor_msgs::msg::LaserScan & scan_msg)
{
  float no_hit_range = use_inf_ ? std::numeric_limits<float>::infinity() :
    scan_msg.range_max - 0.1f;

  if (!has_map_ || !has_initial_pose_ || !has_base_to_laser_) {
    scan_msg.ranges.assign(num_samples, no_hit_range);
    return;
  }

  auto [x0, y0, theta] = getLaserPose();

  double resolution = map_.info.resolution;
  double origin_x = map_.info.origin.position.x;
  double origin_y = map_.info.origin.position.y;
  int width = static_cast<int>(map_.info.width);
  int height = static_cast<int>(map_.info.height);

  int mx0 = static_cast<int>(std::floor((x0 - origin_x) / resolution));
  int my0 = static_cast<int>(std::floor((y0 - origin_y) / resolution));

  if (mx0 <= 0 || mx0 >= width || my0 <= 0 || my0 >= height) {
    scan_msg.ranges.assign(num_samples, no_hit_range);
    return;
  }

  const auto & map_data = map_.data;
  double range_max = scan_msg.range_max;
  double angle_min = scan_msg.angle_min;
  double angle_increment = scan_msg.angle_increment;
  double step = resolution * 0.5;

  for (int i = 0; i < num_samples; ++i) {
    double angle = theta + angle_min + i * angle_increment;
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    scan_msg.ranges[i] = no_hit_range;

    for (double d = 0.0; d <= range_max; d += step) {
      int mx = static_cast<int>(std::floor((x0 + d * cos_a - origin_x) / resolution));
      int my = static_cast<int>(std::floor((y0 + d * sin_a - origin_y) / resolution));
      if (mx <= 0 || mx >= width || my <= 0 || my >= height) {
        break;
      }
      if (map_data[my * width + mx] >= 60) {
        scan_msg.ranges[i] = static_cast<float>(d);
        break;
      }
    }
  }

  // Add Gaussian noise to valid range measurements
  if (scan_noise_std_ > 0.0) {
    std::normal_distribution<float> noise(0.0f, static_cast<float>(scan_noise_std_));
    for (int i = 0; i < num_samples; ++i) {
      float & r = scan_msg.ranges[i];
      if (std::isfinite(r) && r > 0.0f) {
        r = std::max(0.0f, r + noise(rng_));
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult
LoopbackSimulator::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "speed_factor") {
      double factor = param.as_double();
      if (factor <= 0.0) {
        result.successful = false;
        result.reason = "speed_factor must be positive";
        return result;
      }
    }
  }
  return result;
}

void LoopbackSimulator::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "speed_factor") {
      speed_factor_ = param.as_double();
      if (clock_publisher_) {
        clock_publisher_->setSpeedFactor(speed_factor_);
      }
    }
  }
}

}  // namespace nav2_loopback_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_loopback_sim::LoopbackSimulator)
