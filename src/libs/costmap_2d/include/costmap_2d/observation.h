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

#ifndef COSTMAP_2D_OBSERVATION_H_
#define COSTMAP_2D_OBSERVATION_H_

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

namespace costmap_2d
{

/**
 * @brief Stores an observation in terms of a point cloud and the origin of the source
 * @note Tried to make members and constructor arguments const but the compiler would not accept the default
 * assignment operator for vector insertion!
 */
class Observation
{
public:
  /**
   * @brief  Creates an empty observation
   */
  Observation() :
    cloud_(new sensor_msgs::PointCloud2()), obstacle_range_(0.0), raytrace_range_(0.0)
  {
  }

  virtual ~Observation()
  {
    delete cloud_;
  }

  /**
   * @brief  Creates an observation from an origin point and a point cloud
   * @param origin The origin point of the observation
   * @param cloud The point cloud of the observation
   * @param obstacle_range The range out to which an observation should be able to insert obstacles
   * @param raytrace_range The range out to which an observation should be able to clear via raytracing
   */
  Observation(geometry_msgs::Point& origin, const sensor_msgs::PointCloud2 &cloud,
              double obstacle_range, double raytrace_range) :
      origin_(origin), cloud_(new sensor_msgs::PointCloud2(cloud)),
      obstacle_range_(obstacle_range), raytrace_range_(raytrace_range)
  {
  }

  /**
   * @brief  Copy constructor
   * @param obs The observation to copy
   */
  Observation(const Observation& obs) :
      origin_(obs.origin_), cloud_(new sensor_msgs::PointCloud2(*(obs.cloud_))),
      obstacle_range_(obs.obstacle_range_), raytrace_range_(obs.raytrace_range_)
  {
  }

  /**
   * @brief  Creates an observation from a point cloud
   * @param cloud The point cloud of the observation
   * @param obstacle_range The range out to which an observation should be able to insert obstacles
   */
  Observation(const sensor_msgs::PointCloud2 &cloud, double obstacle_range) :
      cloud_(new sensor_msgs::PointCloud2(cloud)), obstacle_range_(obstacle_range), raytrace_range_(0.0)
  {
  }

  geometry_msgs::Point origin_;
  sensor_msgs::PointCloud2* cloud_;
  double obstacle_range_, raytrace_range_;
};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_OBSERVATION_H_
