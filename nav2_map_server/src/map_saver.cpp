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

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{

  public:
    MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free)
      : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);


      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);


      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else if (!strcmp(argv[i], "--occ"))
    {
      if (++i < argc)
      {
        threshold_occupied = std::atoi(argv[i]);
        if (threshold_occupied < 1 || threshold_occupied > 100)
        {
          ROS_ERROR("threshold_occupied must be between 1 and 100");
          return 1;
        }

      }
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else if (!strcmp(argv[i], "--free"))
    {
      if (++i < argc)
      {
        threshold_free = std::atoi(argv[i]);
        if (threshold_free < 0 || threshold_free > 100)
        {
          ROS_ERROR("threshold_free must be between 0 and 100");
          return 1;
        }

      }
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  if (threshold_occupied <= threshold_free)
  {
    ROS_ERROR("threshold_free must be smaller than threshold_occupied");
    return 1;
  }

  MapGenerator mg(mapname, threshold_occupied, threshold_free);

  while(!mg.saved_map_ && ros::ok())
    ros::spinOnce();

  return 0;
}


