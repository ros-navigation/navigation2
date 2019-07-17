/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include "nav2_map_server/map_generator.hpp"

#include <cstdio>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace nav2_map_server
{

/**
 * @brief Map generation node.
 */
MapGenerator::MapGenerator(const std::string & mapname, int threshold_occupied, int threshold_free)
: Node("map_saver"),
  saved_map_(false),
  mapname_(mapname),
  threshold_occupied_(threshold_occupied),
  threshold_free_(threshold_free)
{
  RCLCPP_INFO(get_logger(), "Waiting for the map");
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::SystemDefaultsQoS(),
    std::bind(&MapGenerator::mapCallback, this, std::placeholders::_1));
}

void MapGenerator::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  rclcpp::Logger logger = this->get_logger();
  RCLCPP_INFO(logger, "Received a %d X %d map @ %.3f m/pix",
    map->info.width,
    map->info.height,
    map->info.resolution);


  std::string mapdatafile = mapname_ + ".pgm";
  RCLCPP_INFO(logger, "Writing map occupancy data to %s", mapdatafile.c_str());
  FILE * out = fopen(mapdatafile.c_str(), "w");
  if (!out) {
    RCLCPP_ERROR(logger, "Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

  fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
    map->info.resolution, map->info.width, map->info.height);
  for (unsigned int y = 0; y < map->info.height; y++) {
    for (unsigned int x = 0; x < map->info.width; x++) {
      unsigned int i = x + (map->info.height - y - 1) * map->info.width;
      if (map->data[i] >= 0 && map->data[i] <= threshold_free_) {  // [0,free)
        fputc(254, out);
      } else if (map->data[i] >= threshold_occupied_) {  // (occ,255]
        fputc(000, out);
      } else {  // occ [0.25,0.65]
        fputc(205, out);
      }
    }
  }

  fclose(out);

  std::string mapmetadatafile = mapname_ + ".yaml";
  RCLCPP_INFO(logger, "Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE * yaml = fopen(mapmetadatafile.c_str(), "w");

  geometry_msgs::msg::Quaternion orientation = map->info.origin.orientation;
  tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x,
    orientation.y,
    orientation.z,
    orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\n",
    mapdatafile.c_str(), map->info.resolution,
    map->info.origin.position.x, map->info.origin.position.y, yaw);
  fprintf(yaml, "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n");

  fclose(yaml);

  RCLCPP_INFO(logger, "Done\n");
  saved_map_ = true;
}

}  // namespace nav2_map_server
