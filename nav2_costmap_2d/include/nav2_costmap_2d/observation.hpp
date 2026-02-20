/*
 * Copyright (c) 2008, 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Conor McGann
 */

#ifndef NAV2_COSTMAP_2D__OBSERVATION_HPP_
#define NAV2_COSTMAP_2D__OBSERVATION_HPP_

#include <utility>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/macros.hpp>

namespace nav2_costmap_2d
{

/**
 * @brief Stores an observation in terms of a point cloud and the origin of the source
 */
class Observation
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Observation)

  /**
   * @brief  Creates an empty observation
   */
  Observation() = default;


  /**
   * @brief  Creates an observation from an origin point and a point cloud
   * @param origin The origin point of the observation
   * @param cloud The point cloud of the observation
   * @param obstacle_max_range The range out to which an observation should be able to insert obstacles
   * @param obstacle_min_range The range from which an observation should be able to insert obstacles
   * @param raytrace_max_range The range out to which an observation should be able to clear via raytracing
   * @param raytrace_min_range The range from which an observation should be able to clear via raytracing
   */
  Observation(
    geometry_msgs::msg::Point origin, sensor_msgs::msg::PointCloud2 cloud,
    double obstacle_max_range, double obstacle_min_range, double raytrace_max_range,
    double raytrace_min_range)
  : origin_(std::move(origin)), cloud_(std::move(cloud)),
    obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
    raytrace_max_range_(raytrace_max_range), raytrace_min_range_(
      raytrace_min_range)
  {
  }

  /**
   * @brief  Creates an observation from a point cloud
   * @param cloud The point cloud of the observation
   * @param obstacle_max_range The range out to which an observation should be able to insert obstacles
   * @param obstacle_min_range The range from which an observation should be able to insert obstacles
   */
  Observation(
    const sensor_msgs::msg::PointCloud2 & cloud, double obstacle_max_range,
    double obstacle_min_range)
  : cloud_(std::move(cloud)), obstacle_max_range_(obstacle_max_range),
    obstacle_min_range_(obstacle_min_range)
  {
  }

  geometry_msgs::msg::Point origin_{};
  sensor_msgs::msg::PointCloud2 cloud_{};
  double obstacle_max_range_{0.};
  double obstacle_min_range_{0.};
  double raytrace_max_range_{0.};
  double raytrace_min_range_{0.};
};

}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__OBSERVATION_HPP_
