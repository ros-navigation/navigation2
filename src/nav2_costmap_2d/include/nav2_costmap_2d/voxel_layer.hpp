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
#ifndef NAV2_COSTMAP_2D__VOXEL_LAYER_HPP_
#define NAV2_COSTMAP_2D__VOXEL_LAYER_HPP_

#include <vector>
#include "message_filters/subscriber.h"

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <nav2_voxel_grid/voxel_grid.hpp>

namespace nav2_costmap_2d
{

/**
 * @class VoxelLayer
 * @brief Takes laser and pointcloud data to populate a 3D voxel representation of the environment
 */
class VoxelLayer : public ObstacleLayer
{
public:
  /**
   * @brief Voxel Layer constructor
   */
  VoxelLayer()
  : voxel_grid_(0, 0, 0)
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class's parent class Costmap2D
  }

  /**
   * @brief Voxel Layer destructor
   */
  virtual ~VoxelLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  /**
   * @brief Update the layer's origin to a new pose, often when in a rolling costmap
   */
  void updateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief If layer is discretely populated
   */
  bool isDiscretized()
  {
    return true;
  }

  /**
   * @brief Match the size of the master costmap
   */
  virtual void matchSize();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return true;}

protected:
  /**
   * @brief Reset internal maps
   */
  virtual void resetMaps();

  /**
   * @brief Use raycasting between 2 points to clear freespace
   */
  virtual void raytraceFreespace(
    const nav2_costmap_2d::Observation & clearing_observation,
    double * min_x, double * min_y,
    double * max_x,
    double * max_y);

  bool publish_voxel_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_pub_;
  nav2_voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;
  int unknown_threshold_, mark_threshold_, size_z_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    clearing_endpoints_pub_;

  /**
   * @brief Covert world coordinates into map coordinates
   */
  inline bool worldToMap3DFloat(
    double wx, double wy, double wz, double & mx, double & my,
    double & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }
    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);
    if (mx < size_x_ && my < size_y_ && mz < size_z_) {
      return true;
    }

    return false;
  }

  /**
   * @brief Covert world coordinates into map coordinates
   */
  inline bool worldToMap3D(
    double wx, double wy, double wz, unsigned int & mx, unsigned int & my,
    unsigned int & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    mz = static_cast<unsigned int>((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ && my < size_y_ && mz < (unsigned int)size_z_) {
      return true;
    }

    return false;
  }

  /**
   * @brief Covert map coordinates into world coordinates
   */
  inline void mapToWorld3D(
    unsigned int mx, unsigned int my, unsigned int mz, double & wx,
    double & wy,
    double & wz)
  {
    // returns the center point of the cell
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  }

  /**
   * @brief Find L2 norm distance in 3D
   */
  inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
  }

  /**
   * @brief Get the height of the voxel sizes in meters
   */
  double getSizeInMetersZ() const
  {
    return (size_z_ - 1 + 0.5) * z_resolution_;
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__VOXEL_LAYER_HPP_
