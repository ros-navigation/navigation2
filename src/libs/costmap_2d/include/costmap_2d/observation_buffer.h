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
 *********************************************************************/
#ifndef COSTMAP_2D_OBSERVATION_BUFFER_H_
#define COSTMAP_2D_OBSERVATION_BUFFER_H_

#include <vector>
#include <list>
#include <string>
#include <ros/time.h>
#include <costmap_2d/observation.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/PointCloud2.h>

// Thread support
#include <boost/thread.hpp>

namespace costmap_2d
{
/**
 * @class ObservationBuffer
 * @brief Takes in point clouds from sensors, transforms them to the desired frame, and stores them
 */
class ObservationBuffer
{
public:
  /**
   * @brief  Constructs an observation buffer
   * @param  topic_name The topic of the observations, used as an identifier for error and warning messages
   * @param  observation_keep_time Defines the persistence of observations in seconds, 0 means only keep the latest
   * @param  expected_update_rate How often this buffer is expected to be updated, 0 means there is no limit
   * @param  min_obstacle_height The minimum height of a hitpoint to be considered legal
   * @param  max_obstacle_height The minimum height of a hitpoint to be considered legal
   * @param  obstacle_range The range to which the sensor should be trusted for inserting obstacles
   * @param  raytrace_range The range to which the sensor should be trusted for raytracing to clear out space
   * @param  tf2_buffer A reference to a tf2 Buffer
   * @param  global_frame The frame to transform PointClouds into
   * @param  sensor_frame The frame of the origin of the sensor, can be left blank to be read from the messages
   * @param  tf_tolerance The amount of time to wait for a transform to be available when setting a new global frame
   */
  ObservationBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate,
                    double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                    double raytrace_range, tf2_ros::Buffer& tf2_buffer, std::string global_frame,
                    std::string sensor_frame, double tf_tolerance);

  /**
   * @brief  Destructor... cleans up
   */
  ~ObservationBuffer();

  /**
   * @brief Sets the global frame of an observation buffer. This will
   * transform all the currently cached observations to the new global
   * frame
   * @param new_global_frame The name of the new global frame.
   * @return True if the operation succeeds, false otherwise
   */
  bool setGlobalFrame(const std::string new_global_frame);

  /**
   * @brief  Transforms a PointCloud to the global frame and buffers it
   * <b>Note: The burden is on the user to make sure the transform is available... ie they should use a MessageNotifier</b>
   * @param  cloud The cloud to be buffered
   */
  void bufferCloud(const sensor_msgs::PointCloud2& cloud);

  /**
   * @brief  Pushes copies of all current observations onto the end of the vector passed in
   * @param  observations The vector to be filled
   */
  void getObservations(std::vector<Observation>& observations);

  /**
   * @brief  Check if the observation buffer is being update at its expected rate
   * @return True if it is being updated at the expected rate, false otherwise
   */
  bool isCurrent() const;

  /**
   * @brief  Lock the observation buffer
   */
  inline void lock()
  {
    lock_.lock();
  }

  /**
   * @brief  Lock the observation buffer
   */
  inline void unlock()
  {
    lock_.unlock();
  }

  /**
   * @brief Reset last updated timestamp
   */
  void resetLastUpdated();

private:
  /**
   * @brief  Removes any stale observations from the buffer list
   */
  void purgeStaleObservations();

  tf2_ros::Buffer& tf2_buffer_;
  const ros::Duration observation_keep_time_;
  const ros::Duration expected_update_rate_;
  ros::Time last_updated_;
  std::string global_frame_;
  std::string sensor_frame_;
  std::list<Observation> observation_list_;
  std::string topic_name_;
  double min_obstacle_height_, max_obstacle_height_;
  boost::recursive_mutex lock_;  ///< @brief A lock for accessing data in callbacks safely
  double obstacle_range_, raytrace_range_;
  double tf_tolerance_;
};
}  // namespace costmap_2d
#endif  // COSTMAP_2D_OBSERVATION_BUFFER_H_
