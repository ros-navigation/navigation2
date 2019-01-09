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
#include <cstdlib>
#include <cstring>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_generator.hpp"

#define USAGE "Usage: \n" \
  "  map_saver -h\n" \
  "  map_saver [--occ <threshold_occupied>] [--free <threshold_free>] " \
  "[-f <mapname>] [ROS remapping args]"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger("map_saver");

  std::string mapname = "map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-h")) {
      puts(USAGE);
      return 0;
    } else if (!strcmp(argv[i], "-f")) {
      if (++i < argc) {
        mapname = argv[i];
      } else {
        puts(USAGE);
        return 1;
      }
    } else if (!strcmp(argv[i], "--occ")) {
      if (++i < argc) {
        threshold_occupied = atoi(argv[i]);
        if (threshold_occupied < 1 || threshold_occupied > 100) {
          RCLCPP_ERROR(logger, "Threshold_occupied must be between 1 and 100");
          return 1;
        }
      } else {
        puts(USAGE);
        return 1;
      }
    } else if (!strcmp(argv[i], "--free")) {
      if (++i < argc) {
        threshold_free = atoi(argv[i]);
        if (threshold_free < 0 || threshold_free > 100) {
          RCLCPP_ERROR(logger, "Threshold_free must be between 0 and 100");
          return 1;
        }
      } else {
        puts(USAGE);
        return 1;
      }
    } else {
      puts(USAGE);
      return 1;
    }
  }

  if (threshold_occupied <= threshold_free) {
    RCLCPP_ERROR(logger, "Threshold_free must be smaller than threshold_occupied");
    return 1;
  }

  auto map_gen = std::make_shared<nav2_map_server::MapGenerator>(mapname, threshold_occupied,
      threshold_free);

  while (!map_gen->saved_map_ && rclcpp::ok()) {
    rclcpp::spin_some(map_gen);
    rclcpp::sleep_for(100ms);
  }

  rclcpp::shutdown();
  return 0;
}
