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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <costmap_2d/VoxelGrid.h>
#include <voxel_grid/voxel_grid.h>

struct Cell
{
  double x;
  double y;
  double z;
  voxel_grid::VoxelStatus status;
};
typedef std::vector<Cell> V_Cell;

float g_colors_r[] = {0.0f, 0.0f, 1.0f};
float g_colors_g[] = {0.0f, 0.0f, 0.0f};
float g_colors_b[] = {0.0f, 1.0f, 0.0f};
float g_colors_a[] = {0.0f, 0.5f, 1.0f};

std::string g_marker_ns;
V_Cell g_cells;
void voxelCallback(const ros::Publisher& pub, const costmap_2d::VoxelGridConstPtr& grid)
{
  if (grid->data.empty())
  {
    ROS_ERROR("Received empty voxel grid");
    return;
  }

  ros::WallTime start = ros::WallTime::now();

  ROS_DEBUG("Received voxel grid");
  const std::string frame_id = grid->header.frame_id;
  const ros::Time stamp = grid->header.stamp;
  const uint32_t* data = &grid->data.front();
  const double x_origin = grid->origin.x;
  const double y_origin = grid->origin.y;
  const double z_origin = grid->origin.z;
  const double x_res = grid->resolutions.x;
  const double y_res = grid->resolutions.y;
  const double z_res = grid->resolutions.z;
  const uint32_t x_size = grid->size_x;
  const uint32_t y_size = grid->size_y;
  const uint32_t z_size = grid->size_z;

  g_cells.clear();
  uint32_t num_markers = 0;
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid)
  {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid)
    {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid)
      {
        voxel_grid::VoxelStatus status = voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid, x_size, y_size, z_size,
                                                                         data);

        if (status == voxel_grid::MARKED)
        {
          Cell c;
          c.status = status;
          c.x = x_origin + (x_grid + 0.5) * x_res;
          c.y = y_origin + (y_grid + 0.5) * y_res;
          c.z = z_origin + (z_grid + 0.5) * z_res;
          g_cells.push_back(c);

          ++num_markers;
        }
      }
    }
  }

  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = g_marker_ns;
  m.id = 0;
  m.type = visualization_msgs::Marker::CUBE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = x_res;
  m.scale.y = y_res;
  m.scale.z = z_res;
  m.color.r = g_colors_r[voxel_grid::MARKED];
  m.color.g = g_colors_g[voxel_grid::MARKED];
  m.color.b = g_colors_b[voxel_grid::MARKED];
  m.color.a = g_colors_a[voxel_grid::MARKED];
  m.points.resize(num_markers);
  for (uint32_t i = 0; i < num_markers; ++i)
  {
    Cell& c = g_cells[i];
    geometry_msgs::Point& p = m.points[i];
    p.x = c.x;
    p.y = c.y;
    p.z = c.z;
  }

  pub.publish(m);

  ros::WallTime end = ros::WallTime::now();
  ROS_DEBUG("Published %d markers in %f seconds", num_markers, (end - start).toSec());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_2d_markers");
  ros::NodeHandle n;

  ROS_DEBUG("Startup");

  ros::Publisher pub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe < costmap_2d::VoxelGrid > ("voxel_grid", 1, boost::bind(voxelCallback, pub, _1));
  g_marker_ns = n.resolveName("voxel_grid");

  ros::spin();
}
