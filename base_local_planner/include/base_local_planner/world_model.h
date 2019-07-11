/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
#ifndef TRAJECTORY_ROLLOUT_WORLD_MODEL_H_
#define TRAJECTORY_ROLLOUT_WORLD_MODEL_H_

#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <base_local_planner/planar_laser_scan.h>

namespace base_local_planner {
  /**
   * @class WorldModel
   * @brief An interface the trajectory controller uses to interact with the
   * world regardless of the underlying world model.
   */
  class WorldModel{
    public:
      /**
       * @brief  Subclass will implement this method to check a footprint at a given position and orientation for legality in the world
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise:
       *          -1 if footprint covers at least a lethal obstacle cell, or
       *          -2 if footprint covers at least a no-information cell, or
       *          -3 if footprint is partially or totally outside of the map
       */
      virtual double footprintCost(const geometry_msgs::msg::Point& position, const std::vector<geometry_msgs::msg::Point>& footprint,
          double inscribed_radius, double circumscribed_radius) = 0;

      double footprintCost(double x, double y, double theta, const std::vector<geometry_msgs::msg::Point>& footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius=0.0){

        double cos_th = cos(theta);
        double sin_th = sin(theta);
        std::vector<geometry_msgs::msg::Point> oriented_footprint;
        for(unsigned int i = 0; i < footprint_spec.size(); ++i){
          geometry_msgs::msg::Point new_pt;
          new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
          new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
          oriented_footprint.push_back(new_pt);
        }

        geometry_msgs::msg::Point robot_position;
        robot_position.x = x;
        robot_position.y = y;

        if(inscribed_radius==0.0){
          nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
        }

        return footprintCost(robot_position, oriented_footprint, inscribed_radius, circumscribed_radius);
      }

      /**
       * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise
       */
      double footprintCost(const geometry_msgs::msg::Point& position, const std::vector<geometry_msgs::msg::Point>& footprint,
          double inscribed_radius, double circumscribed_radius, double extra) {
        return footprintCost(position, footprint, inscribed_radius, circumscribed_radius);
      }

      /**
       * @brief  Subclass will implement a destructor
       */
      virtual ~WorldModel(){}

    protected:
      WorldModel(){}
  };

};
#endif
