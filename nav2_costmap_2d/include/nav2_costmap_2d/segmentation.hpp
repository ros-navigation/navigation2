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
 * Authors: Pedro Gonzalez
 */

#ifndef NAV2_COSTMAP_2D__SEGMENTATION_HPP_
#define NAV2_COSTMAP_2D__SEGMENTATION_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace nav2_costmap_2d
{

/**
 * @brief Stores an segmentation in terms of a point cloud and the origin of the source
 * @note Tried to make members and constructor arguments const but the compiler would not accept the
 * default assignment operator for vector insertion!
 */
class Segmentation
{
public:
  /**
   * @brief  Creates an empty segmentation
   */
  Segmentation()
  : cloud_(new sensor_msgs::msg::PointCloud2()) {}
  /**
   * @brief A destructor
   */
  virtual ~Segmentation() {delete cloud_;}

  /**
   * @brief  Copy assignment operator
   * @param obs The segmentation to copy
   */
  Segmentation & operator=(const Segmentation & obs)
  {
    origin_ = obs.origin_;
    cloud_ = new sensor_msgs::msg::PointCloud2(*(obs.cloud_));
    class_map_ = obs.class_map_;

    return *this;
  }

  /**
   * @brief  Creates an segmentation from an origin point and a point cloud
   * @param origin The origin point of the segmentation
   * @param cloud The point cloud of the segmentation
   * @param obstacle_max_range The range out to which an segmentation should be able to insert
   * obstacles
   * @param obstacle_min_range The range from which an segmentation should be able to insert
   * obstacles
   * @param raytrace_max_range The range out to which an segmentation should be able to clear via
   * raytracing
   * @param raytrace_min_range The range from which an segmentation should be able to clear via
   * raytracing
   */
  Segmentation(
    geometry_msgs::msg::Point & origin, const sensor_msgs::msg::PointCloud2 & cloud,
    std::unordered_map<uint16_t, uint8_t> class_map)
  : origin_(origin), cloud_(new sensor_msgs::msg::PointCloud2(cloud)), class_map_(class_map)
  {
  }

  /**
   * @brief  Copy constructor
   * @param obs The segmentation to copy
   */
  Segmentation(const Segmentation & obs)
  : origin_(obs.origin_)
    , cloud_(new sensor_msgs::msg::PointCloud2(*(obs.cloud_)))
    , class_map_(obs.class_map_)
  {
  }

  /**
   * @brief  Creates an segmentation from a point cloud
   * @param cloud The point cloud of the segmentation
   * @param obstacle_max_range The range out to which an segmentation should be able to insert
   * obstacles
   * @param obstacle_min_range The range from which an segmentation should be able to insert
   * obstacles
   */
  Segmentation(const sensor_msgs::msg::PointCloud2 & cloud)
  : cloud_(new sensor_msgs::msg::PointCloud2(cloud))
  {
  }

  geometry_msgs::msg::Point origin_;
  sensor_msgs::msg::PointCloud2 * cloud_;
  std::unordered_map<uint16_t, uint8_t> class_map_; // contains class_id->cost relation
};

}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__SEGMENTATION_HPP_
