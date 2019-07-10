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
#ifndef TRAJECTORY_ROLLOUT_VOXEL_WORLD_MODEL_H_
#define TRAJECTORY_ROLLOUT_VOXEL_WORLD_MODEL_H_
#include <vector>
#include <list>
#include <cfloat>
#include <geometry_msgs/msg/point.hpp>
#include <costmap_2d/observation.h>
#include <base_local_planner/world_model.h>

//voxel grid stuff
#include <voxel_grid/voxel_grid.h>

namespace base_local_planner {
  /**
   * @class VoxelGridModel
   * @brief A class that implements the WorldModel interface to provide grid
   * based collision checks for the trajectory controller using a 3D voxel grid.
   */
  class VoxelGridModel : public WorldModel {
    public:
      /**
       * @brief  Constructor for the VoxelGridModel
       * @param size_x The x size of the map
       * @param size_y The y size of the map
       * @param size_z The z size of the map... up to 32 cells
       * @param xy_resolution The horizontal resolution of the map in meters/cell
       * @param z_resolution The vertical resolution of the map in meters/cell
       * @param origin_x The x value of the origin of the map
       * @param origin_y The y value of the origin of the map
       * @param origin_z The z value of the origin of the map
       * @param  max_z The maximum height for an obstacle to be added to the grid
       * @param  obstacle_range The maximum distance for obstacles to be added to the grid
       */
      VoxelGridModel(double size_x, double size_y, double size_z, double xy_resolution, double z_resolution,
          double origin_x, double origin_y, double origin_z, double max_z, double obstacle_range);

      /**
       * @brief  Destructor for the world model
       */
      virtual ~VoxelGridModel(){}

      /**
       * @brief  Checks if any obstacles in the voxel grid lie inside a convex footprint that is rasterized into the grid
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise
       */
      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      using WorldModel::footprintCost;

      /**
       * @brief  The costmap already keeps track of world observations, so for this world model this method does nothing
       * @param footprint The footprint of the robot in its current location
       * @param observations The observations from various sensors 
       * @param laser_scan The scans used to clear freespace
       */
      void updateWorld(const std::vector<geometry_msgs::Point>& footprint,
          const std::vector<costmap_2d::Observation>& observations, const std::vector<PlanarLaserScan>& laser_scans);

      /**
       * @brief Function copying the Voxel points into a point cloud
       * @param cloud the point cloud to copy data to. It has the usual x,y,z channels
       */
      void getPoints(sensor_msgs::PointCloud2& cloud);

    private:
      /**
       * @brief  Rasterizes a line in the costmap grid and checks for collisions
       * @param x0 The x position of the first cell in grid coordinates
       * @param y0 The y position of the first cell in grid coordinates
       * @param x1 The x position of the second cell in grid coordinates
       * @param y1 The y position of the second cell in grid coordinates
       * @return A positive cost for a legal line... negative otherwise
       */
      double lineCost(int x0, int x1, int y0, int y1);

      /**
       * @brief  Checks the cost of a point in the costmap
       * @param x The x position of the point in cell coordinates 
       * @param y The y position of the point in cell coordinates 
       * @return A positive cost for a legal point... negative otherwise
       */
      double pointCost(int x, int y);

      void removePointsInScanBoundry(const PlanarLaserScan& laser_scan, double raytrace_range);

      inline bool worldToMap3D(double wx, double wy, double wz, unsigned int& mx, unsigned int& my, unsigned int& mz){
        if(wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
          return false;
        mx = (int) ((wx - origin_x_) / xy_resolution_);
        my = (int) ((wy - origin_y_) / xy_resolution_);
        mz = (int) ((wz - origin_z_) / z_resolution_);
        return true;
      }

      inline bool worldToMap2D(double wx, double wy, unsigned int& mx, unsigned int& my){
        if(wx < origin_x_ || wy < origin_y_)
          return false;
        mx = (int) ((wx - origin_x_) / xy_resolution_);
        my = (int) ((wy - origin_y_) / xy_resolution_);
        return true;
      }

      inline void mapToWorld3D(unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy, double& wz){
        //returns the center point of the cell
        wx = origin_x_ + (mx + 0.5) * xy_resolution_;
        wy = origin_y_ + (my + 0.5) * xy_resolution_;
        wz = origin_z_ + (mz + 0.5) * z_resolution_;
      }

      inline void mapToWorld2D(unsigned int mx, unsigned int my, double& wx, double& wy){
        //returns the center point of the cell
        wx = origin_x_ + (mx + 0.5) * xy_resolution_;
        wy = origin_y_ + (my + 0.5) * xy_resolution_;
      }

      inline double dist(double x0, double y0, double z0, double x1, double y1, double z1){
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
      }

      inline void insert(const geometry_msgs::Point32& pt){
        unsigned int cell_x, cell_y, cell_z;
        if(!worldToMap3D(pt.x, pt.y, pt.z, cell_x, cell_y, cell_z))
          return;
        obstacle_grid_.markVoxel(cell_x, cell_y, cell_z);
      }

      voxel_grid::VoxelGrid obstacle_grid_;
      double xy_resolution_;
      double z_resolution_;
      double origin_x_;
      double origin_y_;
      double origin_z_;
      double max_z_;  ///< @brief The height cutoff for adding points as obstacles
      double sq_obstacle_range_;  ///< @brief The square distance at which we no longer add obstacles to the grid

  };
};
#endif
