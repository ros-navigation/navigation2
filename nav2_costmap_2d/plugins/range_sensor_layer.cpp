/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 David V. Lu!!
 *  Copyright (c) 2020, Bytes Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/range_sensor_layer.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::RangeSensorLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using namespace std::literals::chrono_literals;

namespace nav2_costmap_2d
{

RangeSensorLayer::RangeSensorLayer() {}

void RangeSensorLayer::onInitialize()
{
  current_ = true;
  was_reset_ = false;
  buffered_readings_ = 0;
  last_reading_time_ = clock_->now();
  default_value_ = to_cost(0.5);

  matchSize();
  resetRange();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  declareParameter("phi", rclcpp::ParameterValue(1.2));
  node->get_parameter(name_ + "." + "phi", phi_v_);
  declareParameter("inflate_cone", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + "." + "inflate_cone", inflate_cone_);
  declareParameter("no_readings_timeout", rclcpp::ParameterValue(0.0));
  node->get_parameter(name_ + "." + "no_readings_timeout", no_readings_timeout_);
  declareParameter("clear_threshold", rclcpp::ParameterValue(0.2));
  node->get_parameter(name_ + "." + "clear_threshold", clear_threshold_);
  declareParameter("mark_threshold", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
  declareParameter("clear_on_max_reading", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "clear_on_max_reading", clear_on_max_reading_);

  double temp_tf_tol = 0.0;
  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  std::vector<std::string> topic_names{};
  declareParameter("topics", rclcpp::ParameterValue(topic_names));
  node->get_parameter(name_ + "." + "topics", topic_names);

  InputSensorType input_sensor_type = InputSensorType::ALL;
  std::string sensor_type_name;
  declareParameter("input_sensor_type", rclcpp::ParameterValue("ALL"));
  node->get_parameter(name_ + "." + "input_sensor_type", sensor_type_name);

  std::transform(
    sensor_type_name.begin(), sensor_type_name.end(),
    sensor_type_name.begin(), ::toupper);
  RCLCPP_INFO(
    logger_, "%s: %s as input_sensor_type given",
    name_.c_str(), sensor_type_name.c_str());

  if (sensor_type_name == "VARIABLE") {
    input_sensor_type = InputSensorType::VARIABLE;
  } else if (sensor_type_name == "FIXED") {
    input_sensor_type = InputSensorType::FIXED;
  } else if (sensor_type_name == "ALL") {
    input_sensor_type = InputSensorType::ALL;
  } else {
    RCLCPP_ERROR(
      logger_, "%s: Invalid input sensor type: %s. Defaulting to ALL.",
      name_.c_str(), sensor_type_name.c_str());
  }

  // Validate topic names list: it must be a (normally non-empty) list of strings
  if (topic_names.empty()) {
    RCLCPP_FATAL(
      logger_, "Invalid topic names list: it must"
      "be a non-empty list of strings");
    return;
  }

  // Traverse the topic names list subscribing to all of them with the same callback method
  for (auto & topic_name : topic_names) {
    if (input_sensor_type == InputSensorType::VARIABLE) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processVariableRangeMsg, this,
        std::placeholders::_1);
    } else if (input_sensor_type == InputSensorType::FIXED) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processFixedRangeMsg, this,
        std::placeholders::_1);
    } else if (input_sensor_type == InputSensorType::ALL) {
      processRangeMessageFunc_ = std::bind(
        &RangeSensorLayer::processRangeMsg, this,
        std::placeholders::_1);
    } else {
      RCLCPP_ERROR(
        logger_,
        "%s: Invalid input sensor type: %s. Did you make a new type"
        "and forgot to choose the subscriber for it?",
        name_.c_str(), sensor_type_name.c_str());
    }
    range_subs_.push_back(
      node->create_subscription<sensor_msgs::msg::Range>(
        topic_name, rclcpp::SensorDataQoS(), std::bind(
          &RangeSensorLayer::bufferIncomingRangeMsg, this,
          std::placeholders::_1)));

    RCLCPP_INFO(
      logger_, "RangeSensorLayer: subscribed to "
      "topic %s", range_subs_.back()->get_topic_name());
  }
  global_frame_ = layered_costmap_->getGlobalFrameID();
}


double RangeSensorLayer::gamma(double theta)
{
  if (fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    return 1 - pow(theta / max_angle_, 2);
  }
}

double RangeSensorLayer::delta(double phi)
{
  return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

void RangeSensorLayer::get_deltas(double angle, double * dx, double * dy)
{
  double ta = tan(angle);
  if (ta == 0) {
    *dx = 0;
  } else {
    *dx = resolution_ / ta;
  }

  *dx = copysign(*dx, cos(angle));
  *dy = copysign(resolution_, sin(angle));
}

double RangeSensorLayer::sensor_model(double r, double phi, double theta)
{
  double lbda = delta(phi) * gamma(theta);

  double delta = resolution_;

  if (phi >= 0.0 && phi < r - 2 * delta * r) {
    return (1 - lbda) * (0.5);
  } else if (phi < r - delta * r) {
    return lbda * 0.5 * pow((phi - (r - 2 * delta * r)) / (delta * r), 2) +
           (1 - lbda) * .5;
  } else if (phi < r + delta * r) {
    double J = (r - phi) / (delta * r);
    return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else {
    return 0.5;
  }
}

void RangeSensorLayer::bufferIncomingRangeMsg(
  const sensor_msgs::msg::Range::SharedPtr range_message)
{
  range_message_mutex_.lock();
  range_msgs_buffer_.push_back(*range_message);
  range_message_mutex_.unlock();
}

void RangeSensorLayer::updateCostmap()
{
  std::list<sensor_msgs::msg::Range> range_msgs_buffer_copy;

  range_message_mutex_.lock();
  range_msgs_buffer_copy = std::list<sensor_msgs::msg::Range>(range_msgs_buffer_);
  range_msgs_buffer_.clear();
  range_message_mutex_.unlock();

  for (auto & range_msgs_it : range_msgs_buffer_copy) {
    processRangeMessageFunc_(range_msgs_it);
  }
}

void RangeSensorLayer::processRangeMsg(sensor_msgs::msg::Range & range_message)
{
  if (range_message.min_range == range_message.max_range) {
    processFixedRangeMsg(range_message);
  } else {
    processVariableRangeMsg(range_message);
  }
}

void RangeSensorLayer::processFixedRangeMsg(sensor_msgs::msg::Range & range_message)
{
  if (!std::isinf(range_message.range)) {
    RCLCPP_ERROR(
      logger_,
      "Fixed distance ranger (min_range == max_range) in frame %s sent invalid value. "
      "Only -Inf (== object detected) and Inf (== no object detected) are valid.",
      range_message.header.frame_id.c_str());
    return;
  }

  bool clear_sensor_cone = false;

  if (range_message.range > 0) {  // +inf
    if (!clear_on_max_reading_) {
      return;  // no clearing at all
    }
    clear_sensor_cone = true;
  }

  range_message.range = range_message.min_range;

  updateCostmap(range_message, clear_sensor_cone);
}

void RangeSensorLayer::processVariableRangeMsg(sensor_msgs::msg::Range & range_message)
{
  if (range_message.range < range_message.min_range || range_message.range >
    range_message.max_range)
  {
    return;
  }

  bool clear_sensor_cone = false;

  if (range_message.range >= range_message.max_range && clear_on_max_reading_) {
    clear_sensor_cone = true;
  }

  updateCostmap(range_message, clear_sensor_cone);
}

void RangeSensorLayer::updateCostmap(
  sensor_msgs::msg::Range & range_message,
  bool clear_sensor_cone)
{
  max_angle_ = range_message.field_of_view / 2;

  geometry_msgs::msg::PointStamped in, out;
  in.header.stamp = range_message.header.stamp;
  in.header.frame_id = range_message.header.frame_id;

  if (!tf_->canTransform(
      in.header.frame_id, global_frame_, tf2_ros::fromMsg(in.header.stamp)))
  {
    RCLCPP_INFO(
      logger_, "Range sensor layer can't transform from %s to %s",
      global_frame_.c_str(), in.header.frame_id.c_str());
    return;
  }

  tf_->transform(in, out, global_frame_, transform_tolerance_);

  double ox = out.point.x, oy = out.point.y;

  in.point.x = range_message.range;

  tf_->transform(in, out, global_frame_, transform_tolerance_);

  double tx = out.point.x, ty = out.point.y;

  // calculate target props
  double dx = tx - ox, dy = ty - oy, theta = atan2(dy, dx), d = sqrt(dx * dx + dy * dy);

  // Integer Bounds of Update
  int bx0, by0, bx1, by1;

  // Triangle that will be really updated; the other cells within bounds are ignored
  // This triangle is formed by the origin and left and right sides of sonar cone
  int Ox, Oy, Ax, Ay, Bx, By;

  // Bounds includes the origin
  worldToMapNoBounds(ox, oy, Ox, Oy);
  bx1 = bx0 = Ox;
  by1 = by0 = Oy;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update Map with Target Point
  unsigned int aa, ab;
  if (worldToMap(tx, ty, aa, ab)) {
    setCost(aa, ab, 233);
    touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  }

  double mx, my;

  // Update left side of sonar cone
  mx = ox + cos(theta - max_angle_) * d * 1.2;
  my = oy + sin(theta - max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Ax, Ay);
  bx0 = std::min(bx0, Ax);
  bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay);
  by1 = std::max(by1, Ay);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update right side of sonar cone
  mx = ox + cos(theta + max_angle_) * d * 1.2;
  my = oy + sin(theta + max_angle_) * d * 1.2;

  worldToMapNoBounds(mx, my, Bx, By);
  bx0 = std::min(bx0, Bx);
  bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By);
  by1 = std::max(by1, By);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Limit Bounds to Grid
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min(static_cast<int>(size_x_), bx1);
  by1 = std::min(static_cast<int>(size_y_), by1);

  for (unsigned int x = bx0; x <= (unsigned int)bx1; x++) {
    for (unsigned int y = by0; y <= (unsigned int)by1; y++) {
      bool update_xy_cell = true;

      // Unless inflate_cone_ is set to 100 %, we update cells only within the
      // (partially inflated) sensor cone, projected on the costmap as a triangle.
      // 0 % corresponds to just the triangle, but if your sensor fov is very
      // narrow, the covered area can become zero due to cell discretization.
      // See wiki description for more details
      if (inflate_cone_ < 1.0) {
        // Determine barycentric coordinates
        int w0 = orient2d(Ax, Ay, Bx, By, x, y);
        int w1 = orient2d(Bx, By, Ox, Oy, x, y);
        int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

        // Barycentric coordinates inside area threshold; this is not mathematically
        // sound at all, but it works!
        float bcciath = -static_cast<float>(inflate_cone_) * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
      }

      if (update_xy_cell) {
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        update_cell(ox, oy, theta, range_message.range, wx, wy, clear_sensor_cone);
      }
    }
  }

  buffered_readings_++;
  last_reading_time_ = clock_->now();
}

void RangeSensorLayer::update_cell(
  double ox, double oy, double ot, double r,
  double nx, double ny, bool clear)
{
  unsigned int x, y;
  if (worldToMap(nx, ny, x, y)) {
    double dx = nx - ox, dy = ny - oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx * dx + dy * dy);
    double sensor = 0.0;
    if (!clear) {
      sensor = sensor_model(r, phi, theta);
    }
    double prior = to_prob(getCost(x, y));
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    double new_prob = prob_occ / (prob_occ + prob_not);

    RCLCPP_DEBUG(
      logger_,
      "%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    RCLCPP_DEBUG(
      logger_,
      "%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
    unsigned char c = to_cost(new_prob);
    setCost(x, y, c);
  }
}

void RangeSensorLayer::resetRange()
{
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void RangeSensorLayer::updateBounds(
  double robot_x, double robot_y,
  double robot_yaw, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  robot_yaw = 0 + robot_yaw;  // Avoid error if variable not in use
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  updateCostmap();

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  resetRange();

  if (!enabled_) {
    current_ = true;
    return;
  }

  if (buffered_readings_ == 0) {
    if (no_readings_timeout_ > 0.0 &&
      (clock_->now() - last_reading_time_).seconds() >
      no_readings_timeout_)
    {
      RCLCPP_WARN(
        logger_,
        "No range readings received for %.2f seconds, while expected at least every %.2f seconds.",
        (clock_->now() - last_reading_time_).seconds(),
        no_readings_timeout_);
      current_ = false;
    }
  }
}

void RangeSensorLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char current;
      if (prob == nav2_costmap_2d::NO_INFORMATION) {
        it++;
        continue;
      } else if (prob > mark) {
        current = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else if (prob < clear) {
        current = nav2_costmap_2d::FREE_SPACE;
      } else {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      if (old_cost == NO_INFORMATION || old_cost < current) {
        master_array[it] = current;
      }
      it++;
    }
  }

  buffered_readings_ = 0;

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
}

void RangeSensorLayer::reset()
{
  RCLCPP_DEBUG(logger_, "Reseting range sensor layer...");
  deactivate();
  resetMaps();
  was_reset_ = true;
  activate();
}

void RangeSensorLayer::deactivate()
{
  range_msgs_buffer_.clear();
}

void RangeSensorLayer::activate()
{
  range_msgs_buffer_.clear();
}

}  // namespace nav2_costmap_2d
