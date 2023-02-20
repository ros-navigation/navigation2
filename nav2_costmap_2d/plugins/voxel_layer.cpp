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
 *********************************************************************/

#include "nav2_costmap_2d/voxel_layer.hpp"

#include <algorithm>
#include <cassert>
#include <vector>
#include <memory>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#define VOXEL_BITS 16
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VoxelLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

void VoxelLayer::onInitialize()
{
  ObstacleLayer::onInitialize();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
  declareParameter("z_voxels", rclcpp::ParameterValue(10));
  declareParameter("origin_z", rclcpp::ParameterValue(0.0));
  declareParameter("z_resolution", rclcpp::ParameterValue(0.2));
  declareParameter("unknown_threshold", rclcpp::ParameterValue(15));
  declareParameter("mark_threshold", rclcpp::ParameterValue(0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("publish_voxel_map", rclcpp::ParameterValue(false));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "z_voxels", size_z_);
  node->get_parameter(name_ + "." + "origin_z", origin_z_);
  node->get_parameter(name_ + "." + "z_resolution", z_resolution_);
  node->get_parameter(name_ + "." + "unknown_threshold", unknown_threshold_);
  node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter(name_ + "." + "publish_voxel_map", publish_voxel_);

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>(
      "voxel_grid", custom_qos);
    voxel_pub_->on_activate();
  }

  clearing_endpoints_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "clearing_endpoints", custom_qos);
  clearing_endpoints_pub_->on_activate();

  unknown_threshold_ += (VOXEL_BITS - size_z_);
  matchSize();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &VoxelLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
}

VoxelLayer::~VoxelLayer()
{
  dyn_params_handler_.reset();
}

void VoxelLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  assert(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void VoxelLayer::reset()
{
  // Call the base class method before adding our own functionality
  ObstacleLayer::reset();
  resetMaps();
}

void VoxelLayer::resetMaps()
{
  // Call the base class method before adding our own functionality
  // Note: at the time this was written, ObstacleLayer doesn't implement
  // resetMaps so this goes to the next layer down Costmap2DLayer which also
  // doesn't implement this, so it actually goes all the way to Costmap2D
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
}

void VoxelLayer::updateBounds(
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
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = getMarkingObservations(observations) && current;

  // get the clearing observations
  current = getClearingObservations(clearing_observations) && current;

  // update the global current status
  current_ = current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end();
    ++it)
  {
    const Observation & obs = *it;

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // if the obstacle is too high or too far away from the robot we won't add it
      if (*iter_z > max_obstacle_height_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (*iter_x - obs.origin_.x) * (*iter_x - obs.origin_.x) +
        (*iter_y - obs.origin_.y) * (*iter_y - obs.origin_.y) +
        (*iter_z - obs.origin_.z) * (*iter_z - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_max_range) {
        continue;
      }

      // If the point is too close, do not consider it
      if (sq_dist < sq_obstacle_min_range) {
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (*iter_z < origin_z_) {
        if (!worldToMap3D(*iter_x, *iter_y, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(*iter_x, *iter_y, *iter_z, mx, my, mz)) {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        touch(
          static_cast<double>(*iter_x), static_cast<double>(*iter_y),
          min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_) {
    auto grid_msg = std::make_unique<nav2_msgs::msg::VoxelGrid>();
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg->size_x = voxel_grid_.sizeX();
    grid_msg->size_y = voxel_grid_.sizeY();
    grid_msg->size_z = voxel_grid_.sizeZ();
    grid_msg->data.resize(size);
    memcpy(&grid_msg->data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg->origin.x = origin_x_;
    grid_msg->origin.y = origin_y_;
    grid_msg->origin.z = origin_z_;

    grid_msg->resolutions.x = resolution_;
    grid_msg->resolutions.y = resolution_;
    grid_msg->resolutions.z = z_resolution_;
    grid_msg->header.frame_id = global_frame_;
    grid_msg->header.stamp = clock_->now();

    voxel_pub_->publish(std::move(grid_msg));
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void VoxelLayer::raytraceFreespace(
  const Observation & clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  auto clearing_endpoints_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

  if (clearing_observation.cloud_->height == 0 || clearing_observation.cloud_->width == 0) {
    return;
  }

  double sensor_x, sensor_y, sensor_z;
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  double oz = clearing_observation.origin_.z;

  if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z)) {
    RCLCPP_WARN(
      logger_,
      "Sensor origin at (%.2f, %.2f %.2f) is out of map bounds "
      "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f). "
      "The costmap cannot raytrace for it.",
      ox, oy, oz,
      origin_x_, origin_y_, origin_z_,
      origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY(),
      origin_z_ + getSizeInMetersZ());

    return;
  }

  bool publish_clearing_points;

  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    publish_clearing_points = (node->count_subscribers("clearing_endpoints") > 0);
  }

  clearing_endpoints_->data.clear();
  clearing_endpoints_->width = clearing_observation.cloud_->width;
  clearing_endpoints_->height = clearing_observation.cloud_->height;
  clearing_endpoints_->is_dense = true;
  clearing_endpoints_->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(*clearing_endpoints_);
  modifier.setPointCloud2Fields(
    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_x(*clearing_endpoints_, "x");
  sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_y(*clearing_endpoints_, "y");
  sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_z(*clearing_endpoints_, "z");

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double map_end_x = origin_x_ + getSizeInMetersX();
  double map_end_y = origin_y_ + getSizeInMetersY();
  double map_end_z = origin_z_ + getSizeInMetersZ();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*(clearing_observation.cloud_), "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*(clearing_observation.cloud_), "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*(clearing_observation.cloud_), "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double wpx = *iter_x;
    double wpy = *iter_y;
    double wpz = *iter_z;

    double distance = dist(ox, oy, oz, wpx, wpy, wpz);
    double scaling_fact = 1.0;
    scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
    wpx = scaling_fact * (wpx - ox) + ox;
    wpy = scaling_fact * (wpy - oy) + oy;
    wpz = scaling_fact * (wpz - oz) + oz;

    double a = wpx - ox;
    double b = wpy - oy;
    double c = wpz - oz;
    double t = 1.0;

    // we can only raytrace to a maximum z height
    if (wpz > map_end_z) {
      // we know we want the vector's z value to be max_z
      t = std::max(0.0, std::min(t, (map_end_z - 0.01 - oz) / c));
    } else if (wpz < origin_z_) {
      // and we can only raytrace down to the floor
      // we know we want the vector's z value to be 0.0
      t = std::min(t, (origin_z_ - oz) / c);
    }

    // the minimum value to raytrace from is the origin
    if (wpx < origin_x_) {
      t = std::min(t, (origin_x_ - ox) / a);
    }
    if (wpy < origin_y_) {
      t = std::min(t, (origin_y_ - oy) / b);
    }

    // the maximum value to raytrace to is the end of the map
    if (wpx > map_end_x) {
      t = std::min(t, (map_end_x - ox) / a);
    }
    if (wpy > map_end_y) {
      t = std::min(t, (map_end_y - oy) / b);
    }

    wpx = ox + a * t;
    wpy = oy + b * t;
    wpz = oz + c * t;

    double point_x, point_y, point_z;
    if (worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z)) {
      unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
      unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);


      // voxel_grid_.markVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z);
      voxel_grid_.clearVoxelLineInMap(
        sensor_x, sensor_y, sensor_z, point_x, point_y, point_z,
        costmap_,
        unknown_threshold_, mark_threshold_, FREE_SPACE, NO_INFORMATION,
        cell_raytrace_max_range, cell_raytrace_min_range);

      updateRaytraceBounds(
        ox, oy, wpx, wpy, clearing_observation.raytrace_max_range_,
        clearing_observation.raytrace_min_range_, min_x, min_y,
        max_x,
        max_y);

      if (publish_clearing_points) {
        *clearing_endpoints_iter_x = wpx;
        *clearing_endpoints_iter_y = wpy;
        *clearing_endpoints_iter_z = wpz;

        ++clearing_endpoints_iter_x;
        ++clearing_endpoints_iter_y;
        ++clearing_endpoints_iter_z;
      }
    }
  }

  if (publish_clearing_points) {
    clearing_endpoints_->header.frame_id = global_frame_;
    clearing_endpoints_->header.stamp = clearing_observation.cloud_->header.stamp;

    clearing_endpoints_pub_->publish(std::move(clearing_endpoints_));
  }
}

void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // beacuase we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];
  unsigned int * local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int * voxel_map = voxel_grid_.getData();

  // copy the local window in the costmap to the local map
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);
  copyMapRegion(
    voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);

  // we'll reset our maps to unknown space if appropriate
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    cell_size_y);
  copyMapRegion(
    local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_,
    cell_size_x,
    cell_size_y);

  // make sure to clean up
  delete[] local_map;
  delete[] local_voxel_map;
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
VoxelLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;
  bool resize_map_needed = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "max_obstacle_height") {
        max_obstacle_height_ = parameter.as_double();
      } else if (param_name == name_ + "." + "origin_z") {
        origin_z_ = parameter.as_double();
        resize_map_needed = true;
      } else if (param_name == name_ + "." + "z_resolution") {
        z_resolution_ = parameter.as_double();
        resize_map_needed = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
        current_ = false;
      } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      } else if (param_name == name_ + "." + "publish_voxel_map") {
        RCLCPP_WARN(
          logger_, "publish voxel map is not a dynamic parameter "
          "cannot be changed while running. Rejecting parameter update.");
        continue;
      }

    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "z_voxels") {
        size_z_ = parameter.as_int();
        resize_map_needed = true;
      } else if (param_name == name_ + "." + "unknown_threshold") {
        unknown_threshold_ = parameter.as_int() + (VOXEL_BITS - size_z_);
      } else if (param_name == name_ + "." + "mark_threshold") {
        mark_threshold_ = parameter.as_int();
      } else if (param_name == name_ + "." + "combination_method") {
        combination_method_ = parameter.as_int();
      }
    }
  }

  if (resize_map_needed) {
    matchSize();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
