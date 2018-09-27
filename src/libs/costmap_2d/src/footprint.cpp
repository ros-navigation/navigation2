/*
 * Copyright (c) 2013, Willow Garage, Inc.
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
 */

#include<costmap_2d/costmap_math.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <costmap_2d/footprint.h>
#include <costmap_2d/array_parser.h>
#include<geometry_msgs/Point32.h>

namespace costmap_2d
{

void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint, double& min_dist, double& max_dist)
{
  min_dist = std::numeric_limits<double>::max();
  max_dist = 0.0;

  if (footprint.size() <= 2)
  {
    return;
  }

  for (unsigned int i = 0; i < footprint.size() - 1; ++i)
  {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y,
                                      footprint[i + 1].x, footprint[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y,
                                      footprint.front().x, footprint.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt)
{
  geometry_msgs::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

geometry_msgs::Point toPoint(geometry_msgs::Point32 pt)
{
  geometry_msgs::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

geometry_msgs::Polygon toPolygon(std::vector<geometry_msgs::Point> pts)
{
  geometry_msgs::Polygon polygon;
  for (int i = 0; i < pts.size(); i++){
    polygon.points.push_back(toPoint32(pts[i]));
  }
  return polygon;
}

std::vector<geometry_msgs::Point> toPointVector(geometry_msgs::Polygon polygon)
{
  std::vector<geometry_msgs::Point> pts;
  for (int i = 0; i < polygon.points.size(); i++)
  {
    pts.push_back(toPoint(polygon.points[i]));
  }
  return pts;
}

void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        std::vector<geometry_msgs::Point>& oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)
  {
    geometry_msgs::Point new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
}

void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        geometry_msgs::PolygonStamped& oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.polygon.points.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)
  {
    geometry_msgs::Point32 new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

void padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    geometry_msgs::Point& pt = footprint[ i ];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}


std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius)
{
  std::vector<geometry_msgs::Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  geometry_msgs::Point pt;
  for (int i = 0; i < N; ++i)
  {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}


bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprint_string, error);

  if (error != "")
  {
    ROS_ERROR("Error parsing footprint parameter: '%s'", error.c_str());
    ROS_ERROR("  Footprint string was '%s'.", footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3)
  {
    ROS_ERROR("You must specify at least three points for the robot footprint, reverting to previous footprint.");
    return false;
  }
  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[ i ].size() == 2)
    {
      geometry_msgs::Point point;
      point.x = vvf[ i ][ 0 ];
      point.y = vvf[ i ][ 1 ];
      point.z = 0;
      footprint.push_back(point);
    }
    else
    {
      ROS_ERROR("Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                 int(vvf[ i ].size()));
      return false;
    }
  }

  return true;
}



std::vector<geometry_msgs::Point> makeFootprintFromParams(ros::NodeHandle& nh)
{
  std::string full_param_name;
  std::string full_radius_param_name;
  std::vector<geometry_msgs::Point> points;

  if (nh.searchParam("footprint", full_param_name))
  {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam(full_param_name, footprint_xmlrpc);
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
        footprint_xmlrpc != "" && footprint_xmlrpc != "[]")
    {
      if (makeFootprintFromString(std::string(footprint_xmlrpc), points))
      {
        writeFootprintToParam(nh, points);
        return points;
      }
    }
    else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      points = makeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
      writeFootprintToParam(nh, points);
      return points;
    }
  }

  if (nh.searchParam("robot_radius", full_radius_param_name))
  {
    double robot_radius;
    nh.param(full_radius_param_name, robot_radius, 1.234);
    points = makeFootprintFromRadius(robot_radius);
    nh.setParam("robot_radius", robot_radius);
  }
  // Else neither param was found anywhere this knows about, so
  // defaults will come from dynamic_reconfigure stuff, set in
  // cfg/Costmap2D.cfg and read in this file in reconfigureCB().
  return points;
}

void writeFootprintToParam(ros::NodeHandle& nh, const std::vector<geometry_msgs::Point>& footprint)
{
  std::ostringstream oss;
  bool first = true;
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    geometry_msgs::Point p = footprint[ i ];
    if (first)
    {
      oss << "[[" << p.x << "," << p.y << "]";
      first = false;
    }
    else
    {
      oss << ",[" << p.x << "," << p.y << "]";
    }
  }
  oss << "]";
  nh.setParam("footprint", oss.str().c_str());
}

double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                const std::string& full_param_name)
{
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      footprint_xmlrpc.size() < 3)
  {
    ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
               full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                             "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i)
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 2)
    {
      ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                 full_param_name.c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

}  // end namespace costmap_2d
