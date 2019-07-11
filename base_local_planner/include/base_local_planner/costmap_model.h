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
#ifndef TRAJECTORY_ROLLOUT_COSTMAP_MODEL_
#define TRAJECTORY_ROLLOUT_COSTMAP_MODEL_

#include <base_local_planner/world_model.h>
// For obstacle data access
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace base_local_planner {
  /**
   * @class CostmapModel
   * @brief A class that implements the WorldModel interface to provide grid
   * based collision checks for the trajectory controller using the costmap.
   */
  class CostmapModel : public WorldModel {
    public:
      /**
       * @brief  Constructor for the CostmapModel
       * @param costmap The costmap that should be used
       * @return
       */
      CostmapModel(const nav2_costmap_2d::Costmap2D& costmap);

      /**
       * @brief  Destructor for the world model
       */
      virtual ~CostmapModel(){}
      using WorldModel::footprintCost;

      /**
       * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise:
       *            -1 if footprint covers at least a lethal obstacle cell, or
       *            -2 if footprint covers at least a no-information cell, or
       *            -3 if footprint is [partially] outside of the map
       */
      virtual double footprintCost(const geometry_msgs::msg::Point& position, const std::vector<geometry_msgs::msg::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      /**
       * @brief  Rasterizes a line in the costmap grid and checks for collisions
       * @param x0 The x position of the first cell in grid coordinates
       * @param y0 The y position of the first cell in grid coordinates
       * @param x1 The x position of the second cell in grid coordinates
       * @param y1 The y position of the second cell in grid coordinates
       * @return A positive cost for a legal line... negative otherwise
       */
      double lineCost(int x0, int x1, int y0, int y1) const;

      /**
       * @brief  Checks the cost of a point in the costmap
       * @param x The x position of the point in cell coordinates
       * @param y The y position of the point in cell coordinates
       * @return A positive cost for a legal point... negative otherwise
       */
      double pointCost(int x, int y) const;

    private:
      const nav2_costmap_2d::Costmap2D& costmap_; ///< @brief Allows access of costmap obstacle information

  };
};
#endif
