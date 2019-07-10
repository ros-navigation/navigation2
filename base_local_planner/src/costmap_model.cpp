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
#include <base_local_planner/line_iterator.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner {
  CostmapModel::CostmapModel(const Costmap2D& ma) : costmap_(ma) {}

  double CostmapModel::footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
      double inscribed_radius, double circumscribed_radius){
    // returns:
    //  -1 if footprint covers at least a lethal obstacle cell, or
    //  -2 if footprint covers at least a no-information cell, or
    //  -3 if footprint is [partially] outside of the map, or
    //  a positive value for traversable space

    //used to put things into grid coordinates
    unsigned int cell_x, cell_y;

    //get the cell coord of the center point of the robot
    if(!costmap_.worldToMap(position.x, position.y, cell_x, cell_y))
      return -3.0;

    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    if(footprint.size() < 3){
      unsigned char cost = costmap_.getCost(cell_x, cell_y);
      if(cost == NO_INFORMATION)
        return -2.0;
      if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
        return -1.0;
      return cost;
    }

    //now we really have to lay down the footprint in the costmap grid
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    //we need to rasterize each line in the footprint
    for(unsigned int i = 0; i < footprint.size() - 1; ++i){
      //get the cell coord of the first point
      if(!costmap_.worldToMap(footprint[i].x, footprint[i].y, x0, y0))
        return -3.0;

      //get the cell coord of the second point
      if(!costmap_.worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
        return -3.0;

      line_cost = lineCost(x0, x1, y0, y1);
      footprint_cost = std::max(line_cost, footprint_cost);

      //if there is an obstacle that hits the line... we know that we can return false right away
      if(line_cost < 0)
        return line_cost;
    }

    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    if(!costmap_.worldToMap(footprint.back().x, footprint.back().y, x0, y0))
      return -3.0;

    //get the cell coord of the first point
    if(!costmap_.worldToMap(footprint.front().x, footprint.front().y, x1, y1))
      return -3.0;

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    if(line_cost < 0)
      return line_cost;

    //if all line costs are legal... then we can return that the footprint is legal
    return footprint_cost;

  }

  //calculate the cost of a ray-traced line
  double CostmapModel::lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for( LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
    {
      point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

      if(point_cost < 0)
        return point_cost;

      if(line_cost < point_cost)
        line_cost = point_cost;
    }

    return line_cost;
  }

  double CostmapModel::pointCost(int x, int y) const {
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == NO_INFORMATION)
      return -2;
    if(cost == LETHAL_OBSTACLE)
      return -1;

    return cost;
  }

};
