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

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

namespace costmap_2d
{

/**
 * @brief Calculate the extreme distances for the footprint
 *
 * @param footprint The footprint to examine
 * @param min_dist Output parameter of the minimum distance
 * @param max_dist Output parameter of the maximum distance
 */
void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint,
                                 double& min_dist, double& max_dist);

/**
 * @brief Convert Point32 to Point
 */
geometry_msgs::Point              toPoint(geometry_msgs::Point32 pt);

/**
 * @brief Convert Point to Point32
 */
geometry_msgs::Point32            toPoint32(geometry_msgs::Point   pt);

/**
 * @brief Convert vector of Points to Polygon msg
 */
geometry_msgs::Polygon            toPolygon(std::vector<geometry_msgs::Point> pts);

/**
 * @brief Convert Polygon msg to vector of Points.
 */
std::vector<geometry_msgs::Point> toPointVector(geometry_msgs::Polygon polygon);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        std::vector<geometry_msgs::Point>& oriented_footprint);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        geometry_msgs::PolygonStamped & oriented_footprint);

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
void padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding);

/**
 * @brief Create a circular footprint from a given radius
 */
std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius);

/**
 * @brief Make the footprint from the given string.
 *
 * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
 *
 */
bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint);

/**
 * @brief Read the ros-params "footprint" and/or "robot_radius" from
 * the given NodeHandle using searchParam() to go up the tree.
 */
std::vector<geometry_msgs::Point> makeFootprintFromParams(ros::NodeHandle& nh);

/**
 * @brief Create the footprint from the given XmlRpcValue.
 *
 * @param footprint_xmlrpc should be an array of arrays, where the
 * top-level array should have 3 or more elements, and the
 * sub-arrays should all have exactly 2 elements (x and y
 * coordinates).
 *
 * @param full_param_name this is the full name of the rosparam from
 * which the footprint_xmlrpc value came.  It is used only for
 * reporting errors. */
std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                const std::string& full_param_name);

/** @brief Write the current unpadded_footprint_ to the "footprint"
 * parameter of the given NodeHandle so that dynamic_reconfigure
 * will see the new value. */
void writeFootprintToParam(ros::NodeHandle& nh, const std::vector<geometry_msgs::Point>& footprint);

}  // end namespace costmap_2d

#endif  // COSTMAP_2D_FOOTPRINT_H
