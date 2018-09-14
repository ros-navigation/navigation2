/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <costmap_2d/VoxelGrid.h>
#include <voxel_grid/voxel_grid.h>

static inline void mapToWorld3D(const unsigned int mx, const unsigned int my, const unsigned int mz,
                                      const double origin_x, const double origin_y, const double origin_z,
                                      const double x_resolution, const double y_resolution, const double z_resolution,
                                      double& wx, double& wy, double& wz)
{
  // returns the center point of the cell
  wx = origin_x + (mx + 0.5) * x_resolution;
  wy = origin_y + (my + 0.5) * y_resolution;
  wz = origin_z + (mz + 0.5) * z_resolution;
}

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

V_Cell g_marked;
V_Cell g_unknown;
void voxelCallback(const ros::Publisher& pub_marked, const ros::Publisher& pub_unknown,
                   const costmap_2d::VoxelGridConstPtr& grid)
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

  g_marked.clear();
  g_unknown.clear();
  uint32_t num_marked = 0;
  uint32_t num_unknown = 0;
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid)
  {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid)
    {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid)
      {
        voxel_grid::VoxelStatus status = voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid, x_size, y_size, z_size,
                                                                         data);

        if (status == voxel_grid::UNKNOWN)
        {
          Cell c;
          c.status = status;
          mapToWorld3D(x_grid, y_grid, z_grid, x_origin, y_origin, z_origin, x_res, y_res,
                                                   z_res, c.x, c.y, c.z);

          g_unknown.push_back(c);

          ++num_unknown;
        }
        else if (status == voxel_grid::MARKED)
        {
          Cell c;
          c.status = status;
          mapToWorld3D(x_grid, y_grid, z_grid, x_origin, y_origin, z_origin, x_res, y_res,
                                                   z_res, c.x, c.y, c.z);

          g_marked.push_back(c);

          ++num_marked;
        }
      }
    }
  }

  {
    sensor_msgs::PointCloud cloud;
    cloud.points.resize(num_marked);
    cloud.channels.resize(1);
    cloud.channels[0].values.resize(num_marked);
    cloud.channels[0].name = "rgb";
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;

    sensor_msgs::ChannelFloat32& chan = cloud.channels[0];
    for (uint32_t i = 0; i < num_marked; ++i)
    {
      geometry_msgs::Point32& p = cloud.points[i];
      float& cval = chan.values[i];
      Cell& c = g_marked[i];

      p.x = c.x;
      p.y = c.y;
      p.z = c.z;

      uint32_t r = g_colors_r[c.status] * 255.0;
      uint32_t g = g_colors_g[c.status] * 255.0;
      uint32_t b = g_colors_b[c.status] * 255.0;
      // uint32_t a = g_colors_a[c.status] * 255.0;

      uint32_t col = (r << 16) | (g << 8) | b;
      cval = *reinterpret_cast<float*>(&col);
    }

    pub_marked.publish(cloud);
  }

  {
    sensor_msgs::PointCloud cloud;
    cloud.points.resize(num_unknown);
    cloud.channels.resize(1);
    cloud.channels[0].values.resize(num_unknown);
    cloud.channels[0].name = "rgb";
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;

    sensor_msgs::ChannelFloat32& chan = cloud.channels[0];
    for (uint32_t i = 0; i < num_unknown; ++i)
    {
      geometry_msgs::Point32& p = cloud.points[i];
      float& cval = chan.values[i];
      Cell& c = g_unknown[i];

      p.x = c.x;
      p.y = c.y;
      p.z = c.z;

      uint32_t r = g_colors_r[c.status] * 255.0;
      uint32_t g = g_colors_g[c.status] * 255.0;
      uint32_t b = g_colors_b[c.status] * 255.0;
      // uint32_t a = g_colors_a[c.status] * 255.0;

      uint32_t col = (r << 16) | (g << 8) | b;
      cval = *reinterpret_cast<float*>(&col);
    }

    pub_unknown.publish(cloud);
  }

  ros::WallTime end = ros::WallTime::now();
  ROS_DEBUG("Published %d points in %f seconds", num_marked + num_unknown, (end - start).toSec());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_2d_cloud");
  ros::NodeHandle n;

  ROS_DEBUG("Startup");

  ros::Publisher pub_marked = n.advertise < sensor_msgs::PointCloud > ("voxel_marked_cloud", 2);
  ros::Publisher pub_unknown = n.advertise < sensor_msgs::PointCloud > ("voxel_unknown_cloud", 2);
  ros::Subscriber sub = n.subscribe < costmap_2d::VoxelGrid
      > ("voxel_grid", 1, boost::bind(voxelCallback, pub_marked, pub_unknown, _1));

  ros::spin();
}
