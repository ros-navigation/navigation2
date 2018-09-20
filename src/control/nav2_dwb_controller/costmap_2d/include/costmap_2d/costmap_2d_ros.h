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
#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include "costmap_2d.h"
#include <rclcpp/rclcpp.hpp>

namespace costmap_2d
{

class LayeredCostmap{

};

class Costmap2DConfig{

};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
   Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf) {}
  ~Costmap2DROS() {}

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start() {}

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop() {}

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause() {}

  /**
   * @brief  Resumes costmap updates
   */
  void resume(){}

  void updateMap() {}

  /**
   * @brief Reset each individual layer
   */
  void resetLayers() {}

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent() {return false; }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped& global_pose) const {return false;}

  /** @brief Returns costmap name */
  std::string getName() const {return std::string();}

  /** @brief Returns the delay in transform (tf) data that is tolerable in seconds */
  double getTransformTolerance() const {return 0;}

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap(). */
  Costmap2D* getCostmap() {return nullptr;}

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  std::string getGlobalFrameID() {return std::string();}

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  std::string getBaseFrameID() {return std::string();}
  LayeredCostmap* getLayeredCostmap() {return nullptr;}

  /** @brief Returns the current padded footprint as a geometry_msgs::msg::Polygon. */
  geometry_msgs::msg::Polygon getRobotFootprintPolygon() {return geometry_msgs::msg::Polygon();}

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::msg::Point> getRobotFootprint() {return std::vector<geometry_msgs::msg::Point>();}

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::msg::Point> getUnpaddedRobotFootprint() {return std::vector<geometry_msgs::msg::Point>();}

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::msg::Point>& oriented_footprint) const {}



};
// class Costmap2DROS
#pragma GCC diagnostic pop

}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
