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
#include <costmap_2d/observation_buffer.h>

#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace tf;

namespace costmap_2d
{
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, TransformListener& tf, string global_frame,
                                     string sensor_frame, double tf_tolerance) :
    tf_(tf), observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    last_updated_(ros::Time::now()), global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance)
{
}

ObservationBuffer::~ObservationBuffer()
{
}

bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  if (!tf_.waitForTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_),
                            ros::Duration(0.01), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }

  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    try
    {
      Observation& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      tf_.transformPoint(new_global_frame, origin, origin);
      obs.origin_ = origin.point;

      // we also need to transform the cloud of the observation to the new global frame
      pcl_ros::transformPointCloud(new_global_frame, *obs.cloud_, *obs.cloud_, tf_);
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame_.c_str(),
                new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  try
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    // Actually convert the PointCloud2 message into a type we can reason about
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    bufferCloud(pcl_cloud);
  }
  catch (pcl::PCLException& ex)
  {
    ROS_ERROR("Failed to convert a message to a pcl type, dropping observation: %s", ex.what());
    return;
  }
}

void ObservationBuffer::bufferCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  Stamped < tf::Vector3 > global_origin;

  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try
  {
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    Stamped < tf::Vector3 > local_origin(tf::Vector3(0, 0, 0),
                            pcl_conversions::fromPCL(cloud.header).stamp, origin_frame);
    tf_.waitForTransform(global_frame_, local_origin.frame_id_, local_origin.stamp_, ros::Duration(0.5));
    tf_.transformPoint(global_frame_, local_origin, global_origin);
    observation_list_.front().origin_.x = global_origin.getX();
    observation_list_.front().origin_.y = global_origin.getY();
    observation_list_.front().origin_.z = global_origin.getZ();

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;

    pcl::PointCloud < pcl::PointXYZ > global_frame_cloud;

    // transform the point cloud
    pcl_ros::transformPointCloud(global_frame_, cloud, global_frame_cloud, tf_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below or above our height thresholds
    pcl::PointCloud < pcl::PointXYZ > &observation_cloud = *(observation_list_.front().cloud_);
    unsigned int cloud_size = global_frame_cloud.points.size();
    observation_cloud.points.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    for (unsigned int i = 0; i < cloud_size; ++i)
    {
      if (global_frame_cloud.points[i].z <= max_obstacle_height_
          && global_frame_cloud.points[i].z >= min_obstacle_height_)
      {
        observation_cloud.points[point_count++] = global_frame_cloud.points[i];
      }
    }

    // resize the cloud for the number of legal points
    observation_cloud.points.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  }
  catch (TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    observations.push_back(*obs_it);
  }
}

void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == ros::Duration(0.0))
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      Observation& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      ros::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
      if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    ROS_WARN(
        "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = ros::Time::now();
}
}  // namespace costmap_2d

