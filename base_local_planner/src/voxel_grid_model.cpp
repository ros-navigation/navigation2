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
#include <base_local_planner/voxel_grid_model.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner {
  VoxelGridModel::VoxelGridModel(double size_x, double size_y, double size_z, double xy_resolution, double z_resolution,
          double origin_x, double origin_y, double origin_z, double max_z, double obstacle_range) :
    obstacle_grid_(size_x, size_y, size_z), xy_resolution_(xy_resolution), z_resolution_(z_resolution), 
    origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z),
    max_z_(max_z), sq_obstacle_range_(obstacle_range * obstacle_range) {}

  double VoxelGridModel::footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint, 
      double inscribed_radius, double circumscribed_radius){
    if(footprint.size() < 3)
      return -1.0;

    //now we really have to lay down the footprint in the costmap grid
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;

    //we need to rasterize each line in the footprint
    for(unsigned int i = 0; i < footprint.size() - 1; ++i){
      //get the cell coord of the first point
      if(!worldToMap2D(footprint[i].x, footprint[i].y, x0, y0))
        return -1.0;

      //get the cell coord of the second point
      if(!worldToMap2D(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
        return -1.0;

      line_cost = lineCost(x0, x1, y0, y1);

      //if there is an obstacle that hits the line... we know that we can return false right away 
      if(line_cost < 0)
        return -1.0;
    }

    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    if(!worldToMap2D(footprint.back().x, footprint.back().y, x0, y0))
      return -1.0;

    //get the cell coord of the first point
    if(!worldToMap2D(footprint.front().x, footprint.front().y, x1, y1))
      return -1.0;

    line_cost = lineCost(x0, x1, y0, y1);

    if(line_cost < 0)
      return -1.0;

    //if all line costs are legal... then we can return that the footprint is legal
    return 0.0;
  }

  //calculate the cost of a ray-traced line
  double VoxelGridModel::lineCost(int x0, int x1, 
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      point_cost = pointCost(x, y); //Score the current point

      if(point_cost < 0)
        return -1;

      if(line_cost < point_cost)
        line_cost = point_cost;

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double VoxelGridModel::pointCost(int x, int y){
    //if the cell is in an obstacle the path is invalid
    if(obstacle_grid_.getVoxelColumn(x, y)){
      return -1;
    }

    return 1;
  }

  void VoxelGridModel::updateWorld(const std::vector<geometry_msgs::Point>& footprint, 
      const vector<Observation>& observations, const vector<PlanarLaserScan>& laser_scans){

    //remove points in the laser scan boundry
    for(unsigned int i = 0; i < laser_scans.size(); ++i)
      removePointsInScanBoundry(laser_scans[i], 10.0);

    //iterate through all observations and update the grid
    for(vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it){
      const Observation& obs = *it;
      const sensor_msgs::PointCloud2& cloud = *(obs.cloud_);

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

      geometry_msgs::Point32 pt;

      for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
        //filter out points that are too high
        if(*iter_z > max_z_)
          continue;

        //compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (*iter_x - obs.origin_.x) * (*iter_x - obs.origin_.x)
          + (*iter_y - obs.origin_.y) * (*iter_y - obs.origin_.y)
          + (*iter_z - obs.origin_.z) * (*iter_z - obs.origin_.z);

        if(sq_dist >= sq_obstacle_range_)
          continue;

        //insert the point
        pt.x = *iter_x;
        pt.y = *iter_y;
        pt.z = *iter_z;
        insert(pt);
      }
    }

    //remove the points that are in the footprint of the robot
    //removePointsInPolygon(footprint);
  }

  void VoxelGridModel::removePointsInScanBoundry(const PlanarLaserScan& laser_scan, double raytrace_range){
    if(laser_scan.cloud.points.size() == 0)
      return;

    unsigned int sensor_x, sensor_y, sensor_z;
    double ox = laser_scan.origin.x;
    double oy = laser_scan.origin.y;
    double oz = laser_scan.origin.z;
    
    if(!worldToMap3D(ox, oy, oz, sensor_x, sensor_y, sensor_z))
      return;

    for(unsigned int i = 0; i < laser_scan.cloud.points.size(); ++i){
      double wpx = laser_scan.cloud.points[i].x;
      double wpy = laser_scan.cloud.points[i].y;
      double wpz = laser_scan.cloud.points[i].z;

      double distance = dist(ox, oy, oz, wpx, wpy, wpz);
      double scaling_fact = raytrace_range / distance;
      scaling_fact = scaling_fact > 1.0 ? 1.0 : scaling_fact;
      wpx = scaling_fact * (wpx - ox) + ox;
      wpy = scaling_fact * (wpy - oy) + oy;
      wpz = scaling_fact * (wpz - oz) + oz;

      //we can only raytrace to a maximum z height
      if(wpz >= max_z_){
        //we know we want the vector's z value to be max_z
        double a = wpx - ox;
        double b = wpy - oy;
        double c = wpz - oz;
        double t = (max_z_ - .01 - oz) / c;
        wpx = ox + a * t;
        wpy = oy + b * t;
        wpz = oz + c * t;
      }
      //and we can only raytrace down to the floor
      else if(wpz < 0.0){
        //we know we want the vector's z value to be 0.0
        double a = wpx - ox;
        double b = wpy - oy;
        double c = wpz - oz;
        double t = (0.0 - oz) / c;
        wpx = ox + a * t;
        wpy = oy + b * t;
        wpz = oz + c * t;
      }

      unsigned int point_x, point_y, point_z;
      if(worldToMap3D(wpx, wpy, wpz, point_x, point_y, point_z)){
        obstacle_grid_.clearVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z);
      }
    }
  }

  void VoxelGridModel::getPoints(sensor_msgs::PointCloud2& cloud){
    size_t n = 0;

    for(unsigned int i = 0; i < obstacle_grid_.sizeX(); ++i)
      for(unsigned int j = 0; j < obstacle_grid_.sizeY(); ++j)
        for(unsigned int k = 0; k < obstacle_grid_.sizeZ(); ++k)
          if(obstacle_grid_.getVoxel(i, j, k))
            ++n;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(n);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for(unsigned int i = 0; i < obstacle_grid_.sizeX(); ++i){
      for(unsigned int j = 0; j < obstacle_grid_.sizeY(); ++j){
        for(unsigned int k = 0; k < obstacle_grid_.sizeZ(); ++k){
          if(obstacle_grid_.getVoxel(i, j, k)){
            double wx, wy, wz;
            mapToWorld3D(i, j, k, wx, wy, wz);
            *iter_x = wx;
            *iter_y = wy;
            *iter_z = wz;
            ++iter_x;
            ++iter_y;
            ++iter_z;
          }
        }
      }
    }
  }
};
