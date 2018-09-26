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
#ifndef COSTMAP_2D_FOOTPRINT_H
#define COSTMAP_2D_FOOTPRINT_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace costmap_2d
{

/**
 * @brief Calculate the extreme distances for the footprint
 *
 * @param footprint The footprint to examine
 * @param min_dist Output parameter of the minimum distance
 * @param max_dist Output parameter of the maximum distance
 */
inline void calculateMinAndMaxDistances(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double & min_dist, double & max_dist) {}

/**
 * @brief Convert Point32 to Point
 */
inline geometry_msgs::msg::Point              toPoint(geometry_msgs::msg::Point32 pt)
{
  return geometry_msgs::msg::Point();
}

/**
 * @brief Convert Point to Point32
 */
inline geometry_msgs::msg::Point32            toPoint32(geometry_msgs::msg::Point pt)
{
  return geometry_msgs::msg::Point32();
}

/**
 * @brief Convert vector of Points to Polygon msg
 */
inline geometry_msgs::msg::Polygon            toPolygon(std::vector<geometry_msgs::msg::Point> pts)
{
  return geometry_msgs::msg::Polygon();
}

/**
 * @brief Convert Polygon msg to vector of Points.
 */
inline std::vector<geometry_msgs::msg::Point> toPointVector(geometry_msgs::msg::Polygon polygon)
{
  return std::vector<geometry_msgs::msg::Point>();
}

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
inline void transformFootprint(
  double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  std::vector<geometry_msgs::msg::Point> & oriented_footprint) {}

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
inline void transformFootprint(
  double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  geometry_msgs::msg::PolygonStamped & oriented_footprint) {}

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
inline void padFootprint(std::vector<geometry_msgs::msg::Point> & footprint, double padding) {}

/**
 * @brief Create a circular footprint from a given radius
 */
inline std::vector<geometry_msgs::msg::Point> makeFootprintFromRadius(double radius)
{
  return std::vector<geometry_msgs::msg::Point>();
}

/**
 * @brief Make the footprint from the given string.
 *
 * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
 *
 */
inline bool makeFootprintFromString(
  const std::string & footprint_string,
  std::vector<geometry_msgs::msg::Point> & footprint)
{
  return false;
}

/**
 * @brief Read the ros-params "footprint" and/or "robot_radius" from
 * the given NodeHandle using searchParam() to go up the tree.
 */
inline std::vector<geometry_msgs::msg::Point> makeFootprintFromParams(rclcpp::Node::SharedPtr nh)
{
  return std::vector<geometry_msgs::msg::Point>();
}

/** @brief Write the current unpadded_footprint_ to the "footprint"
 * parameter of the given NodeHandle so that dynamic_reconfigure
 * will see the new value. */
inline void writeFootprintToParam(
  rclcpp::Node::SharedPtr nh,
  const std::vector<geometry_msgs::msg::Point> & footprint) {}

}  // end namespace costmap_2d

#endif  // COSTMAP_2D_FOOTPRINT_H
