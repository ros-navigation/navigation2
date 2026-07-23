// Copyright (c) 2026 Dexory
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

#include "nav2_collision_monitor/exclusion_zone.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_collision_monitor/polygon_utils.hpp"

namespace nav2_collision_monitor
{

ExclusionZone::ExclusionZone(
  const nav2::LifecycleNode::WeakPtr & node,
  const std::string & zone_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const bool base_shift_correction)
: node_(node), zone_name_(zone_name), tf_buffer_(tf_buffer),
  base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
  transform_tolerance_(transform_tolerance), base_shift_correction_(base_shift_correction),
  min_height_(-std::numeric_limits<double>::max()),
  max_height_(std::numeric_limits<double>::max())
{
  RCLCPP_INFO(logger_, "[%s]: Creating ExclusionZone", zone_name_.c_str());
}

ExclusionZone::~ExclusionZone()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying ExclusionZone", zone_name_.c_str());
  zone_pub_.reset();
  poly_.clear();
  node_clock_.reset();
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}

bool ExclusionZone::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node_clock_ = node->get_clock();

  if (!getParameters()) {
    return false;
  }

  if (visualize_) {
    zone_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "~/" + zone_name_);
  }

  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(&ExclusionZone::updateParametersCallback, this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ExclusionZone::validateParameterUpdatesCallback, this, std::placeholders::_1));

  return true;
}

bool ExclusionZone::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  enabled_ = node->declare_or_get_parameter(zone_name_ + ".enabled", false);
  visualize_ = node->declare_or_get_parameter(zone_name_ + ".visualize", false);

  // Frame the zone is anchored to. Empty -> static zone in the robot base frame.
  frame_id_ = node->declare_or_get_parameter(
    zone_name_ + ".frame_id", base_frame_id_);
  if (frame_id_.empty()) {
    frame_id_ = base_frame_id_;
  }

  frame_hold_timeout_ = node->declare_or_get_parameter(
    zone_name_ + ".frame_hold_timeout", 0.0);
  // Optional height band (in base frame). Unbounded by default so 2D sources are covered.
  min_height_ = node->declare_or_get_parameter(
    zone_name_ + ".min_height", -std::numeric_limits<double>::max());
  max_height_ = node->declare_or_get_parameter(
    zone_name_ + ".max_height", std::numeric_limits<double>::max());

  const std::string type = node->declare_or_get_parameter(
    zone_name_ + ".type", std::string("polygon"));

  if (type == "circle") {
    is_circle_ = true;
    radius_ = node->declare_or_get_parameter(zone_name_ + ".radius", -1.0);
    if (radius_ <= 0.0) {
      RCLCPP_ERROR(
        logger_, "[%s]: circle exclusion zone requires a positive 'radius'",
        zone_name_.c_str());
      return false;
    }
    radius_squared_ = radius_ * radius_;
  } else if (type == "polygon") {
    is_circle_ = false;
    // Polygon vertices as a VVF string "[[x1, y1], [x2, y2], ...]" expressed in
    // frame_id_, matching the format used by the collision-monitor action polygons.
    const std::string points_str = node->declare_or_get_parameter(
      zone_name_ + ".points", std::string());
    std::string error;
    if (!parsePolygonPoints(points_str, 3, poly_, error)) {
      RCLCPP_ERROR(
        logger_, "[%s]: %s", zone_name_.c_str(), error.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(
      logger_, "[%s]: unknown exclusion zone type: %s",
      zone_name_.c_str(), type.c_str());
    return false;
  }

  return true;
}

void ExclusionZone::apply(const rclcpp::Time & curr_time, std::vector<Point> & data) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enabled_ || data.empty()) {
    return;
  }

  // Resolve the zone-frame -> base-frame transform for this cycle. A flaky zone
  // frame is held at its last known world pose until the hold timeout expires,
  // after which we fail safe and keep all points.
  tf2::Transform tf_zone_to_base;
  if (!getZoneToBaseTransform(curr_time, tf_zone_to_base)) {
    return;
  }

  if (is_circle_) {
    // Circle center is the origin of the zone frame, expressed in base frame.
    const tf2::Vector3 center = tf_zone_to_base.getOrigin();
    const double cx = center.x();
    const double cy = center.y();
    data.erase(
      std::remove_if(
        data.begin(), data.end(),
        [&](const Point & p) {
          if (p.z < min_height_ || p.z > max_height_) {
            return false;
          }
          const double dx = p.x - cx;
          const double dy = p.y - cy;
          return (dx * dx + dy * dy) <= radius_squared_;
        }),
      data.end());
  } else {
    // Transform the polygon vertices into the base frame once.
    std::vector<Point> poly_base;
    transformPolygonPoints(tf_zone_to_base, poly_, poly_base);
    data.erase(
      std::remove_if(
        data.begin(), data.end(),
        [&](const Point & p) {
          if (p.z < min_height_ || p.z > max_height_) {
            return false;
          }
          return nav2_util::geometry_utils::isPointInsidePolygon(p.x, p.y, poly_base);
        }),
      data.end());
  }
}

bool ExclusionZone::getZoneToBaseTransform(
  const rclcpp::Time & curr_time, tf2::Transform & tf_zone_to_base) const
{
  // Zone anchored to the robot base: it rides with the robot, nothing to bridge.
  if (frame_id_ == base_frame_id_) {
    tf_zone_to_base.setIdentity();
    return true;
  }

  // Runs on the safety thread, so all lookups are NON-BLOCKING (zero timeout):
  // they read the cached TF buffer and fail immediately. A blocking lookup on a
  // flaky zone frame would stall the monitor loop, make healthy sources look
  // stale and trip the source_timeout watchdog. transform_tolerance_ is used only
  // as the staleness allowance below, never as a wait.
  const tf2::Duration non_blocking = tf2::Duration::zero();

  // Look up the zone frame at the *latest* available time (never curr_time) so a
  // slowly published frame is not extrapolated; its stamp tells us how stale it is.
  rclcpp::Time zone_stamp = curr_time;
  tf2::Transform tf_zone_to_global;
  tf_zone_to_global.setIdentity();
  if (frame_id_ != global_frame_id_) {
    geometry_msgs::msg::TransformStamped zone_to_global_msg;
    if (!nav2_util::getTransform(
        frame_id_, global_frame_id_, non_blocking, tf_buffer_, zone_to_global_msg))
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_clock_, 2000,
        "[%s]: no transform available for exclusion zone frame '%s'; not excluding any points",
        zone_name_.c_str(), frame_id_.c_str());
      return false;
    }
    zone_stamp = rclcpp::Time(zone_to_global_msg.header.stamp, curr_time.get_clock_type());

    // Accept the pose only within the hold window: at least the transform tolerance
    // (so a healthy frame always passes), extended by frame_hold_timeout_ to ride
    // out brief dropouts. Anything older is dropped so we are never blinded by a
    // frame that may have moved.
    const double age = (curr_time - zone_stamp).seconds();
    const double max_age =
      std::max(frame_hold_timeout_, tf2::durationToSec(transform_tolerance_));
    if (age > max_age) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_clock_, 2000,
        "[%s]: exclusion zone frame '%s' stale for %.2fs (> %.2fs hold window); "
        "not excluding any points",
        zone_name_.c_str(), frame_id_.c_str(), age, max_age);
      return false;
    }

    // Freeze the pose in the smooth global (odom) frame -- where the last valid
    // detection placed the zone in the WORLD. The charger frame is a child of a
    // robot lidar frame, so a held zone-to-base transform would otherwise ride
    // with the robot and sweep the mask off the world-fixed charger.
    tf2::fromMsg(zone_to_global_msg.transform, tf_zone_to_global);
  }

  // Re-project the (possibly held) world pose into the current base frame by
  // advancing only the global -> base leg. base_shift_correction samples the base
  // at curr_time; otherwise the latest base pose is used. Both stay non-blocking.
  tf2::Transform tf_global_to_base;
  bool got_base;
  if (base_shift_correction_) {
    got_base = nav2_util::getTransform(
      global_frame_id_, curr_time, base_frame_id_, curr_time, global_frame_id_,
      non_blocking, tf_buffer_, tf_global_to_base);
  } else {
    got_base = nav2_util::getTransform(
      global_frame_id_, base_frame_id_, non_blocking, tf_buffer_, tf_global_to_base);
  }
  if (!got_base) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_clock_, 2000,
      "[%s]: cannot transform '%s' -> '%s'; not excluding any points",
      zone_name_.c_str(), global_frame_id_.c_str(), base_frame_id_.c_str());
    return false;
  }

  tf_zone_to_base = tf_global_to_base * tf_zone_to_global;
  return true;
}

std::string ExclusionZone::getName() const
{
  return zone_name_;
}

bool ExclusionZone::getEnabled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

void ExclusionZone::activate()
{
  if (zone_pub_) {
    zone_pub_->on_activate();
  }
}

void ExclusionZone::deactivate()
{
  if (zone_pub_) {
    zone_pub_->on_deactivate();
  }
}

void ExclusionZone::publish() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!zone_pub_ || !enabled_) {
    return;
  }

  // Resolve the same (possibly held) zone -> base transform the mask uses and
  // publish the polygon in the smooth base frame. Publishing in the raw zone
  // frame_id_ is unreliable during a dropout (it rides with the robot while
  // frozen); failing here also mirrors the mask going inactive.
  tf2::Transform tf_zone_to_base;
  if (!getZoneToBaseTransform(node_clock_->now(), tf_zone_to_base)) {
    return;
  }

  const std::vector<Point> & vertices = is_circle_ ? circleToPolygon(radius_) : poly_;
  std::vector<Point> vertices_base;
  transformPolygonPoints(tf_zone_to_base, vertices, vertices_base);

  auto msg = std::make_unique<geometry_msgs::msg::PolygonStamped>();
  msg->header.frame_id = base_frame_id_;
  msg->header.stamp = node_clock_->now();
  for (const Point & v : vertices_base) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(v.x);
    p.y = static_cast<float>(v.y);
    msg->polygon.points.push_back(p);
  }

  zone_pub_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult ExclusionZone::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_name = parameter.get_name();
    if (param_name.find(zone_name_ + ".") != 0) {
      continue;
    }
    if (param_name == zone_name_ + ".radius" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
      parameter.as_double() <= 0.0)
    {
      result.successful = false;
      result.reason = "radius must be > 0";
    } else if (param_name == zone_name_ + ".frame_hold_timeout" &&  // NOLINT
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
      parameter.as_double() < 0.0)
    {
      result.successful = false;
      result.reason = "frame_hold_timeout must be >= 0";
    } else if (param_name == zone_name_ + ".points" &&  // NOLINT
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      // Reject a live polygon update that cannot be parsed into a valid polygon
      // so the running zone is never left with a malformed shape.
      std::vector<Point> parsed;
      std::string error;
      if (!parsePolygonPoints(parameter.as_string(), 3, parsed, error)) {
        result.successful = false;
        result.reason = error;
      }
    }
  }
  return result;
}

void ExclusionZone::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & parameter : parameters) {
    const auto & param_name = parameter.get_name();
    if (param_name.find(zone_name_ + ".") != 0) {
      continue;
    }
    if (param_name == zone_name_ + ".enabled" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      enabled_ = parameter.as_bool();
    } else if (param_name == zone_name_ + ".radius" &&  // NOLINT
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      radius_ = parameter.as_double();
      radius_squared_ = radius_ * radius_;
    } else if (param_name == zone_name_ + ".min_height" &&  // NOLINT
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      min_height_ = parameter.as_double();
    } else if (param_name == zone_name_ + ".max_height" &&  // NOLINT
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      max_height_ = parameter.as_double();
    } else if (param_name == zone_name_ + ".frame_hold_timeout" &&  // NOLINT
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      frame_hold_timeout_ = parameter.as_double();
    } else if (param_name == zone_name_ + ".points" && !is_circle_ &&  // NOLINT
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      // The update callback runs only after validation succeeded, so the string
      // is guaranteed to parse into a valid polygon here.
      std::vector<Point> parsed;
      std::string error;
      if (parsePolygonPoints(parameter.as_string(), 3, parsed, error)) {
        poly_ = parsed;
      }
    }
  }
}

}  // namespace nav2_collision_monitor
