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
#ifndef POINT_GRID_H_
#define POINT_GRID_H_
#include <vector>
#include <list>
#include <cfloat>
#include <geometry_msgs/msg/point.hpp>
//#include "geometry_msgs/msg/point.hpp"
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <base_local_planner/world_model.h>

#include <sensor_msgs/msg/point_cloud.h>

namespace base_local_planner {
  /**
   * @class PointGrid
   * @brief A class that implements the WorldModel interface to provide
   * free-space collision checks for the trajectory controller. This class
   * stores points binned into a grid and performs point-in-polygon checks when
   * necessary to determine the legality of a footprint at a given
   * position/orientation.
   */
  class PointGrid : public WorldModel {
    public:
      /**
       * @brief  Constuctor for a grid that stores points in the plane
       * @param  width The width in meters of the grid
       * @param  height The height in meters of the gird
       * @param  resolution The resolution of the grid in meters/cell
       * @param  origin The origin of the bottom left corner of the grid
       * @param  max_z The maximum height for an obstacle to be added to the grid
       * @param  obstacle_range The maximum distance for obstacles to be added to the grid
       * @param  min_separation The minimum distance between points in the grid
       */
      PointGrid(double width, double height, double resolution, geometry_msgs::msg::Point origin,
          double max_z, double obstacle_range, double min_separation);

      /**
       * @brief  Destructor for a point grid
       */
      virtual ~PointGrid(){}

      /**
       * @brief  Returns the points that lie within the cells contained in the specified range. Some of these points may be outside the range itself.
       * @param  lower_left The lower left corner of the range search
       * @param  upper_right The upper right corner of the range search
       * @param points A vector of pointers to lists of the relevant points
       */
      void getPointsInRange(const geometry_msgs::msg::Point& lower_left, const geometry_msgs::msg::Point& upper_right, std::vector< std::list<geometry_msgs::msg::Point32>* >& points);

      /**
       * @brief  Checks if any points in the grid lie inside a convex footprint
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise
       */
      virtual double footprintCost(const geometry_msgs::msg::Point& position, const std::vector<geometry_msgs::msg::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      using WorldModel::footprintCost;

      /**
       * @brief  Inserts observations from sensors into the point grid
       * @param footprint The footprint of the robot in its current location
       * @param observations The observations from various sensors
       * @param laser_scans The laser scans used to clear freespace (the point grid only uses the first scan which is assumed to be the base laser)
       */
      void updateWorld(const std::vector<geometry_msgs::msg::Point>& footprint,
          const std::vector<nav2_costmap_2d::Observation>& observations, const std::vector<PlanarLaserScan>& laser_scans);

      /**
       * @brief  Convert from world coordinates to grid coordinates
       * @param  pt A point in world space
       * @param  gx The x coordinate of the corresponding grid cell to be set by the function
       * @param  gy The y coordinate of the corresponding grid cell to be set by the function
       * @return True if the conversion was successful, false otherwise
       */
      inline bool gridCoords(geometry_msgs::msg::Point pt, unsigned int& gx, unsigned int& gy) const {
        if(pt.x < origin_.x || pt.y < origin_.y){
          gx = 0;
          gy = 0;
          return false;
        }
        gx = (int) ((pt.x - origin_.x)/resolution_);
        gy = (int) ((pt.y - origin_.y)/resolution_);

        if(gx >= width_ || gy >= height_){
          gx = 0;
          gy = 0;
          return false;
        }

        return true;
      }

      /**
       * @brief  Get the bounds in world coordinates of a cell in the point grid, assumes a legal cell when called
       * @param  gx The x coordinate of the grid cell
       * @param  gy The y coordinate of the grid cell
       * @param  lower_left The lower left bounds of the cell in world coordinates to be filled in
       * @param  upper_right The upper right bounds of the cell in world coordinates to be filled in
       */
      inline void getCellBounds(unsigned int gx, unsigned int gy, geometry_msgs::msg::Point& lower_left, geometry_msgs::msg::Point& upper_right) const {
        lower_left.x = gx * resolution_ + origin_.x;
        lower_left.y = gy * resolution_ + origin_.y;

        upper_right.x = lower_left.x + resolution_;
        upper_right.y = lower_left.y + resolution_;
      }


      /**
       * @brief  Compute the squared distance between two points
       * @param pt1 The first point
       * @param pt2 The second point
       * @return The squared distance between the two points
       */
      inline double sq_distance(const geometry_msgs::msg::Point32& pt1, const geometry_msgs::msg::Point32& pt2){
        return (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y);
      }

      /**
       * @brief  Convert from world coordinates to grid coordinates
       * @param  pt A point in world space
       * @param  gx The x coordinate of the corresponding grid cell to be set by the function
       * @param  gy The y coordinate of the corresponding grid cell to be set by the function
       * @return True if the conversion was successful, false otherwise
       */
      inline bool gridCoords(const geometry_msgs::msg::Point32& pt, unsigned int& gx, unsigned int& gy) const {
        if(pt.x < origin_.x || pt.y < origin_.y){
          gx = 0;
          gy = 0;
          return false;
        }
        gx = (int) ((pt.x - origin_.x)/resolution_);
        gy = (int) ((pt.y - origin_.y)/resolution_);

        if(gx >= width_ || gy >= height_){
          gx = 0;
          gy = 0;
          return false;
        }

        return true;
      }

      /**
       * @brief  Converts cell coordinates to an index value that can be used to look up the correct grid cell
       * @param gx The x coordinate of the cell
       * @param gy The y coordinate of the cell
       * @return The index of the cell in the stored cell list
       */
      inline unsigned int gridIndex(unsigned int gx, unsigned int gy) const {
        /*
         * (0, 0) ---------------------- (width, 0)
         *  |                               |
         *  |                               |
         *  |                               |
         *  |                               |
         *  |                               |
         * (0, height) ----------------- (width, height)
         */
        return(gx + gy * width_);
      }

      /**
       * @brief  Check the orientation of a pt c with respect to the vector a->b
       * @param a The start point of the vector
       * @param b The end point of the vector
       * @param c The point to compute orientation for
       * @return orient(a, b, c) < 0 ----> Right, orient(a, b, c) > 0 ----> Left
       */
      inline double orient(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b, const geometry_msgs::msg::Point32& c){
        double acx = a.x - c.x;
        double bcx = b.x - c.x;
        double acy = a.y - c.y;
        double bcy = b.y - c.y;
        return acx * bcy - acy * bcx;
      }

      /**
       * @brief  Check the orientation of a pt c with respect to the vector a->b
       * @param a The start point of the vector
       * @param b The end point of the vector
       * @param c The point to compute orientation for
       * @return orient(a, b, c) < 0 ----> Right, orient(a, b, c) > 0 ----> Left
       */
      template<typename T>
      inline double orient(const T& a, const T& b, const T& c){
        double acx = a.x - c.x;
        double bcx = b.x - c.x;
        double acy = a.y - c.y;
        double bcy = b.y - c.y;
        return acx * bcy - acy * bcx;
      }

      /**
       * @brief  Check if two line segmenst intersect
       * @param v1 The first point of the first segment
       * @param v2 The second point of the first segment
       * @param u1 The first point of the second segment
       * @param u2 The second point of the second segment
       * @return True if the segments intersect, false otherwise
       */
      inline bool segIntersect(const geometry_msgs::msg::Point32& v1, const geometry_msgs::msg::Point32& v2,
          const geometry_msgs::msg::Point32& u1, const geometry_msgs::msg::Point32& u2){
        return (orient(v1, v2, u1) * orient(v1, v2, u2) < 0) && (orient(u1, u2, v1) * orient(u1, u2, v2) < 0);
      }

      /**
       * @brief  Find the intersection point of two lines
       * @param v1 The first point of the first segment
       * @param v2 The second point of the first segment
       * @param u1 The first point of the second segment
       * @param u2 The second point of the second segment
       * @param result The point to be filled in
       */
      void intersectionPoint(const geometry_msgs::msg::Point& v1, const geometry_msgs::msg::Point& v2,
          const geometry_msgs::msg::Point& u1, const geometry_msgs::msg::Point& u2,
          geometry_msgs::msg::Point& result);

      /**
       * @brief  Check if a point is in a polygon
       * @param pt The point to be checked
       * @param poly The polygon to check against
       * @return True if the point is in the polygon, false otherwise
       */
      bool ptInPolygon(const geometry_msgs::msg::Point32& pt, const std::vector<geometry_msgs::msg::Point>& poly);

      /**
       * @brief  Insert a point into the point grid
       * @param pt The point to be inserted
       */
      void insert(const geometry_msgs::msg::Point32& pt);

      /**
       * @brief  Find the distance between a point and its nearest neighbor in the grid
       * @param pt The point used for comparison
       * @return  The distance between the point passed in and its nearest neighbor in the point grid
       */
      double nearestNeighborDistance(const geometry_msgs::msg::Point32& pt);

      /**
       * @brief  Find the distance between a point and its nearest neighbor in a cell
       * @param pt The point used for comparison
       * @param gx The x coordinate of the cell
       * @param gy The y coordinate of the cell
       * @return  The distance between the point passed in and its nearest neighbor in the cell
       */
      double getNearestInCell(const geometry_msgs::msg::Point32& pt, unsigned int gx, unsigned int gy);

      /**
       * @brief  Removes points from the grid that lie within the polygon
       * @param poly A specification of the polygon to clear from the grid
       */
      void removePointsInPolygon(const std::vector<geometry_msgs::msg::Point> poly);

      /**
       * @brief  Removes points from the grid that lie within a laser scan
       * @param  laser_scan A specification of the laser scan to use for clearing
       */
      void removePointsInScanBoundry(const PlanarLaserScan& laser_scan);

      /**
       * @brief  Checks to see if a point is within a laser scan specification
       * @param  pt The point to check
       * @param  laser_scan The specification of the scan to check against
       * @return True if the point is contained within the scan, false otherwise
       */
      bool ptInScan(const geometry_msgs::msg::Point32& pt, const PlanarLaserScan& laser_scan);

      /**
       * @brief  Get the points in the point grid
       * @param  cloud The point cloud to insert the points into
       */
      void getPoints(sensor_msgs::msg::PointCloud2& cloud);

    private:
      double resolution_; ///< @brief The resolution of the grid in meters/cell
      geometry_msgs::msg::Point origin_; ///< @brief The origin point of the grid
      unsigned int width_; ///< @brief The width of the grid in cells
      unsigned int height_; ///< @brief The height of the grid in cells
      std::vector< std::list<geometry_msgs::msg::Point32> > cells_; ///< @brief Storage for the cells in the grid
      double max_z_;  ///< @brief The height cutoff for adding points as obstacles
      double sq_obstacle_range_;  ///< @brief The square distance at which we no longer add obstacles to the grid
      double sq_min_separation_;  ///< @brief The minimum square distance required between points in the grid
      std::vector< std::list<geometry_msgs::msg::Point32>* > points_;  ///< @brief The lists of points returned by a range search, made a member to save on memory allocation
  };
};
#endif
