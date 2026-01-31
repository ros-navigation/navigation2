/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Steve Macenski
 *********************************************************************/
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_util/raytrace_line_2d.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/interface_factories.hpp"
#include "rclcpp/version.h"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

ObstacleLayer::~ObstacleLayer()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
  for (auto & notifier : observation_notifiers_) {
    notifier.reset();
  }
}

void ObstacleLayer::onInitialize()
{
  bool track_unknown_space;
  double transform_tolerance = 0.1;

  // The topics that we'll subscribe to from the parameter server
  std::string topics_string;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  allow_parameter_qos_overrides_ = nav2::declare_or_get_parameter(node,
    "allow_parameter_qos_overrides", true);
  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);
  footprint_clearing_enabled_ = node->declare_or_get_parameter(
    name_ + "." + "footprint_clearing_enabled", true);
  min_obstacle_height_ = node->declare_or_get_parameter(
    name_ + "." + "min_obstacle_height", 0.0);
  max_obstacle_height_ = node->declare_or_get_parameter(
    name_ + "." + "max_obstacle_height", 2.0);
  int combination_method_param = node->declare_or_get_parameter(
    name_ + "." + "combination_method", 1);
  topics_string = node->declare_or_get_parameter(
    name_ + "." + "observation_sources", std::string(""));
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);
  double tf_filter_tolerance = nav2::declare_or_get_parameter(
    node, name_ + "." +
    "tf_filter_tolerance", 0.05);
  combination_method_ = combination_method_from_int(combination_method_param);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ObstacleLayer::dynamicParametersCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Subscribed to Topics: %s", topics_string.c_str());

  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  ObstacleLayer::matchSize();
  current_ = true;
  was_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source) {
    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type, transport_type;
    bool inf_is_valid, clearing, marking;

    topic = node->declare_or_get_parameter(
      name_ + "." + source + "." + "topic", source);
    sensor_frame = node->declare_or_get_parameter(
      name_ + "." + source + "." + "sensor_frame", std::string(""));
    observation_keep_time = node->declare_or_get_parameter(
      name_ + "." + source + "." + "observation_persistence", 0.0);
    expected_update_rate = node->declare_or_get_parameter(
      name_ + "." + source + "." + "expected_update_rate", 0.0);
    data_type = node->declare_or_get_parameter(
      name_ + "." + source + "." + "data_type", std::string("LaserScan"));
    min_obstacle_height = node->declare_or_get_parameter(
      name_ + "." + source + "." + "min_obstacle_height", 0.0);
    max_obstacle_height = node->declare_or_get_parameter(
      name_ + "." + source + "." + "max_obstacle_height", 0.0);
    inf_is_valid = node->declare_or_get_parameter(
      name_ + "." + source + "." + "inf_is_valid", false);
    marking = node->declare_or_get_parameter(
      name_ + "." + source + "." + "marking", true);
    clearing = node->declare_or_get_parameter(
      name_ + "." + source + "." + "clearing", false);
    transport_type = node->declare_or_get_parameter(
      name_ + "." + source + "." + "transport_type", std::string("raw"));

    if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
      RCLCPP_FATAL(
        logger_,
        "Only topics that use point cloud2s or laser scans are currently supported");
      throw std::runtime_error(
              "Only topics that use point cloud2s or laser scans are currently supported");
    }

    // get the obstacle range for the sensor
    double obstacle_max_range = node->declare_or_get_parameter(
      name_ + "." + source + "." + "obstacle_max_range", 2.5);
    double obstacle_min_range = node->declare_or_get_parameter(
      name_ + "." + source + "." + "obstacle_min_range", 0.0);

    // get the raytrace ranges for the sensor
    double raytrace_max_range = node->declare_or_get_parameter(
      name_ + "." + source + "." + "raytrace_max_range", 3.0);
    double raytrace_min_range = node->declare_or_get_parameter(
      name_ + "." + source + "." + "raytrace_min_range", 0.0);

    topic = joinWithParentNamespace(topic);

    RCLCPP_DEBUG(
      logger_,
      "Creating an observation buffer for source %s, topic %s, frame %s",
      source.c_str(), topic.c_str(),
      sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
          std::make_shared<ObservationBuffer>(node, topic, observation_keep_time,
        expected_update_rate,
          min_obstacle_height,
          max_obstacle_height, obstacle_max_range, obstacle_min_range, raytrace_max_range,
          raytrace_min_range, *tf_,
          global_frame_,
          sensor_frame, tf2::durationFromSec(transform_tolerance)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }

    RCLCPP_DEBUG(
      logger_,
      "Created an observation buffer for source %s, topic %s, global frame: %s, "
      "expected update rate: %.2f, observation persistence: %.2f",
      source.c_str(), topic.c_str(),
      global_frame_.c_str(), expected_update_rate, observation_keep_time);

    const auto custom_qos_profile = nav2::qos::SensorDataQoS(50);

    // create a callback for the topic
    if (data_type == "LaserScan") {
      auto sub_opt = nav2::interfaces::createSubscriptionOptions(
        topic, allow_parameter_qos_overrides_, callback_group_);

      // For Kilted and Older Support from Message Filters API change
      #if RCLCPP_VERSION_GTE(29, 6, 0)
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> sub;
      #else
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
        rclcpp_lifecycle::LifecycleNode>> sub;
      #endif

      // For Kilted compatibility in Message Filters API change
      #if RCLCPP_VERSION_GTE(29, 6, 0)
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        node, topic, custom_qos_profile, sub_opt);
      // For Jazzy compatibility in Message Filters API change
      #elif RCLCPP_VERSION_GTE(29, 0, 0)
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
          rclcpp_lifecycle::LifecycleNode>>(
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node),
        topic, custom_qos_profile, sub_opt);
      // For Humble and Older compatibility in Message Filters API change
      #else
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
          rclcpp_lifecycle::LifecycleNode>>(
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node),
        topic, custom_qos_profile.get_rmw_qos_profile(), sub_opt);
      #endif

      sub->unsubscribe();

      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *sub, *tf_, global_frame_, 50,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      if (inf_is_valid) {
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanValidInfCallback, this, std::placeholders::_1,
            observation_buffers_.back()));

      } else {
        filter->registerCallback(
          std::bind(
            &ObstacleLayer::laserScanCallback, this, std::placeholders::_1,
            observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);

      observation_notifiers_.push_back(filter);
      observation_notifiers_.back()->setTolerance(
        rclcpp::Duration::from_seconds(
          tf_filter_tolerance));

    } else {
      auto sub_opt = nav2::interfaces::createSubscriptionOptions(
        topic, allow_parameter_qos_overrides_, callback_group_);

      // For Rolling and Newer Support from PointCloudTransport API change
      #if RCLCPP_VERSION_GTE(30, 0, 0)
      std::shared_ptr<point_cloud_transport::SubscriberFilter> sub;
      // For Kilted and Older Support from Message Filters API change
      #elif RCLCPP_VERSION_GTE(29, 6, 0)
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub;
      #else
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
        rclcpp_lifecycle::LifecycleNode>> sub;
      #endif

      // For Rolling compatibility in PointCloudTransport API change
      #if RCLCPP_VERSION_GTE(30, 0, 0)
      sub = std::make_shared<point_cloud_transport::SubscriberFilter>(
        *node, topic, transport_type, custom_qos_profile, sub_opt);
      // For Kilted compatibility in Message Filters API change
      #elif RCLCPP_VERSION_GTE(29, 6, 0)
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        node, topic, custom_qos_profile, sub_opt);
      // For Jazzy compatibility in Message Filters API change
      #elif RCLCPP_VERSION_GTE(29, 0, 0)
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
          rclcpp_lifecycle::LifecycleNode>>(
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node),
        topic, custom_qos_profile, sub_opt);
      // For Humble and Older compatibility in Message Filters API change
      #else
      sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
          rclcpp_lifecycle::LifecycleNode>>(
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node),
        topic, custom_qos_profile.get_rmw_qos_profile(), sub_opt);
      #endif

      sub->unsubscribe();

      if (inf_is_valid) {
        RCLCPP_WARN(
          logger_,
          "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      auto filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *sub, *tf_, global_frame_, 50,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(transform_tolerance));

      filter->registerCallback(
        std::bind(
          &ObstacleLayer::pointCloud2Callback, this, std::placeholders::_1,
          observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "") {
      std::vector<std::string> target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }
}

rcl_interfaces::msg::SetParametersResult
ObstacleLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "min_obstacle_height") {
        min_obstacle_height_ = parameter.as_double();
      } else if (param_name == name_ + "." + "max_obstacle_height") {
        max_obstacle_height_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = combination_method_from_int(parameter.as_int());
      }
    }
  }

  result.successful = true;
  return result;
}

void
ObstacleLayer::laserScanCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message,
  const std::shared_ptr<ObservationBuffer> & buffer)
{
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(),
      ex.what());
    projector_.projectLaser(*message, cloud);
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::laserScanValidInfCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
  const std::shared_ptr<ObservationBuffer> & buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::msg::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
      global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  } catch (std::runtime_error & ex) {
    RCLCPP_WARN(
      logger_,
      "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
      " Ignore this message. what(): %s",
      ex.what());
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void
ObstacleLayer::pointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<ObservationBuffer> & buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void
ObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation::ConstSharedPtr> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // raytrace freespace
  for (const auto & clearing_observation : clearing_observations) {
    raytraceFreespace(*clearing_observation, min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (const auto & observation : observations) {
    const Observation & obs = *observation;

    const sensor_msgs::msg::PointCloud2 & cloud = obs.cloud_;

    const unsigned int max_range_cells = cellDistance(obs.obstacle_max_range_);
    const unsigned int min_range_cells = cellDistance(obs.obstacle_min_range_);

    unsigned int x0, y0;
    if (!worldToMap(obs.origin_.x, obs.origin_.y, x0, y0)) {
      RCLCPP_DEBUG(logger_, "Sensor origin is out of map bounds");
      continue;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too low, we won't add it
      if (pz < min_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too low");
        continue;
      }

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        RCLCPP_DEBUG(logger_, "The point is too high");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      // compute the distance from the hitpoint to the pointcloud's origin
      // Calculate the distance in cell space to match the ray trace algorithm
      // used for clearing obstacles (see Costmap2D::raytraceLine).
      const int dx = static_cast<int>(mx) - static_cast<int>(x0);
      const int dy = static_cast<int>(my) - static_cast<int>(y0);
      const unsigned int dist = static_cast<unsigned int>(
        std::hypot(static_cast<double>(dx), static_cast<double>(dy)));

      // if the point is far enough away... we won't consider it
      if (dist > max_range_cells) {
        RCLCPP_DEBUG(logger_, "The point is too far away");
        continue;
      }

      // if the point is too close, do not consider it
      if (dist < min_range_cells) {
        RCLCPP_DEBUG(logger_, "The point is too close");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
ObstacleLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
ObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  switch (combination_method_) {
    case CombinationMethod::Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::Max:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::MaxWithoutUnknownOverwrite:
      updateWithMaxWithoutUnknownOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void
ObstacleLayer::addStaticObservation(
  nav2_costmap_2d::Observation obs,
  bool marking, bool clearing)
{
  const auto observation = Observation::make_shared(std::move(obs));
  if (marking) {
    static_marking_observations_.push_back(observation);
  }
  if (clearing) {
    static_clearing_observations_.push_back(observation);
  }
}

void
ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking) {
    static_marking_observations_.clear();
  }
  if (clearing) {
    static_clearing_observations_.clear();
  }
}

bool
ObstacleLayer::getMarkingObservations(
  std::vector<Observation::ConstSharedPtr> & marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (const auto & marking_buffer : marking_buffers_) {
    if (marking_buffer) {
      marking_buffer->lock();
      marking_buffer->getObservations(marking_observations);
      current = marking_buffer->isCurrent() && current;
      marking_buffer->unlock();
    }
  }
  marking_observations.insert(
    marking_observations.end(),
    static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool
ObstacleLayer::getClearingObservations(
  std::vector<Observation::ConstSharedPtr> & clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (const auto & clearing_buffer : clearing_buffers_) {
    if (clearing_buffer) {
      clearing_buffer->lock();
      clearing_buffer->getObservations(clearing_observations);
      current = clearing_buffer->isCurrent() && current;
      clearing_buffer->unlock();
    }
  }
  clearing_observations.insert(
    clearing_observations.end(),
    static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void
ObstacleLayer::raytraceFreespace(
  const Observation & clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::msg::PointCloud2 & cloud = clearing_observation.cloud_;

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_WARN(
      logger_,
      "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). "
      "The costmap cannot raytrace for it.",
      ox, oy,
      origin_x_, origin_y_,
      origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
    return;
  }

  // we can pre-compute the endpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin
  // and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the endpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1)) {
      continue;
    }

    unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    nav2_util::raytraceLine(
      marker, x0, y0, x1, y1, size_x_, cell_raytrace_max_range, cell_raytrace_min_range);

    updateRaytraceBounds(
      ox, oy, wx, wy, clearing_observation.raytrace_max_range_,
      clearing_observation.raytrace_min_range_, min_x, min_y, max_x,
      max_y);
  }
}

void
ObstacleLayer::activate()
{
  for (auto & notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
  resetBuffersLastUpdated();
}

void
ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

void
ObstacleLayer::updateRaytraceBounds(
  double ox, double oy, double wx, double wy, double max_range, double min_range,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  if (full_distance < min_range) {
    return;
  }
  double scale = std::min(1.0, max_range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void
ObstacleLayer::reset()
{
  resetMaps();
  resetBuffersLastUpdated();
  current_ = false;
  was_reset_ = true;
}

void
ObstacleLayer::resetBuffersLastUpdated()
{
  for (const auto & observation_buffer : observation_buffers_) {
    if (observation_buffer) {
      observation_buffer->resetLastUpdated();
    }
  }
}

}  // namespace nav2_costmap_2d
