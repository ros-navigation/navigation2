// Copyright 2019 Rover Robotics
// Copyright (c) 2008, Willow Garage, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "nav2_map_server/map_2d/map_mode.hpp"
#include "nav2_map_server/map_2d/map_saver_2d.hpp"
#include "nav2_map_server/map_3d/map_saver_3d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace nav2_map_server;  // NOLINT

const char * USAGE_STRING{
  "Usage:\n"
  "  map_saver_cli [arguments] [--ros-args ROS remapping args]\n"
  "\n"
  "Arguments:\n"
  "  -h/--help\n"
  "\n-----Parameters common to 2D and 3D map saver-----\n\n"
  "  -t <map_topic>\n"
  "  -f <map_name>\n"
  "  --fmt <map_format> (Supported Formats: <pgm/bmp/png> for 2D and <pcd> for 3D)\n"
  "\n-----Parameters unique to 2D map saver-----\n\n"
  "  --occ <threshold_occupied>\n"
  "  --free <threshold_free>\n"
  "  --mode trinary(default)/scale/raw\n"
  "\n-----Parameters unique to 3D map saver-----\n\n"
  "  --origin <origin_topic>\n"
  "  --as_bin Give the flag to save map with binary encodings\n"
  "\n"
  "NOTE: --ros-args should be passed at the end of command line"};

typedef enum
{
  COMMAND_MAP_TOPIC,
  COMMAND_MAP_FILE_NAME,
  COMMAND_IMAGE_FORMAT,
  COMMAND_OCCUPIED_THRESH,
  COMMAND_FREE_THRESH,
  COMMAND_MODE,
  COMMAND_ENCODING,
  COMMAND_VIEW_POINT_TOPIC
} COMMAND_TYPE;

struct cmd_struct
{
  const char * cmd;
  COMMAND_TYPE command_type;
};

typedef enum
{
  ARGUMENTS_INVALID,
  ARGUMENTS_VALID,
  HELP_MESSAGE
} ARGUMENTS_STATUS;

struct SaveParamList
{
  // 2D parameters
  map_2d::SaveParameters save_parameters_2d;
  // 3D parameters
  map_3d::SaveParameters save_parameters_3d;
  std::string origin_topic;
};

// Arguments parser
// Input parameters: logger, argc, argv
// Output parameters: map_topic, save_parameters
ARGUMENTS_STATUS parse_arguments(
  const rclcpp::Logger & logger, int argc, char ** argv,
  std::string & map_topic, SaveParamList & save_parameters)
{
  const struct cmd_struct commands[] = {
    {"-t", COMMAND_MAP_TOPIC},
    {"-f", COMMAND_MAP_FILE_NAME},
    {"--occ", COMMAND_OCCUPIED_THRESH},
    {"--free", COMMAND_FREE_THRESH},
    {"--mode", COMMAND_MODE},
    {"--as_bin", COMMAND_ENCODING},
    {"--origin", COMMAND_VIEW_POINT_TOPIC},
    {"--fmt", COMMAND_IMAGE_FORMAT}
  };

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;


  size_t cmd_size = sizeof(commands) / sizeof(commands[0]);
  size_t i;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h" || *it == "--help") {
      std::cout << USAGE_STRING << std::endl;
      return HELP_MESSAGE;
    }
    if (*it == "--ros-args") {
      break;
    }
    for (i = 0; i < cmd_size; i++) {
      if (commands[i].cmd == *it) {
        if ((it + 1) == arguments.end()) {
          RCLCPP_ERROR(logger, "Wrong argument: %s should be followed by a value.", it->c_str());
          return ARGUMENTS_INVALID;
        }
        it++;
        switch (commands[i].command_type) {
          case COMMAND_MAP_TOPIC:
            map_topic = *it;
            break;
          case COMMAND_MAP_FILE_NAME:
            save_parameters.save_parameters_2d.map_file_name = *it;
            save_parameters.save_parameters_3d.map_file_name = *it;
            break;
          case COMMAND_FREE_THRESH:
            save_parameters.save_parameters_2d.free_thresh = atoi(it->c_str());
            break;
          case COMMAND_OCCUPIED_THRESH:
            save_parameters.save_parameters_2d.occupied_thresh = atoi(it->c_str());
            break;
          case COMMAND_IMAGE_FORMAT:
            save_parameters.save_parameters_2d.image_format = *it;
            save_parameters.save_parameters_3d.format = *it;
            break;
          case COMMAND_MODE:
            try {
              save_parameters.save_parameters_2d.mode = map_2d::map_mode_from_string(*it);
            } catch (std::invalid_argument &) {
              save_parameters.save_parameters_2d.mode = map_2d::MapMode::Trinary;
              RCLCPP_WARN(
                logger,
                "Map mode parameter not recognized: %s, using default value (trinary)",
                it->c_str());
            }
            break;
          case COMMAND_VIEW_POINT_TOPIC:
            save_parameters.origin_topic = *it;
            break;
          case COMMAND_ENCODING:
            it--;  // as this one is a simple flag that puts binary format to on
            save_parameters.save_parameters_3d.as_binary = true;
            break;
        }
        break;
      }
    }
    if (i == cmd_size) {
      RCLCPP_ERROR(logger, "Wrong argument: %s", it->c_str());
      return ARGUMENTS_INVALID;
    }
  }

  return ARGUMENTS_VALID;
}

int main(int argc, char ** argv)
{
  // ROS2 init
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("map_saver_cli");

  // Parse CLI-arguments
  SaveParamList save_parameters;
  std::string map_topic = "map";
  save_parameters.origin_topic = "map_origin";
  switch (parse_arguments(logger, argc, argv, map_topic, save_parameters)) {
    case ARGUMENTS_INVALID:
      rclcpp::shutdown();
      return -1;
    case HELP_MESSAGE:
      rclcpp::shutdown();
      return 0;
    case ARGUMENTS_VALID:
      break;
  }

  // Call saveMapTopicToFile()
  int retcode;
  try {
    if (save_parameters.save_parameters_3d.format == "pcd" &&
      save_parameters.save_parameters_3d.format == "ply")
    {
      auto map_saver = std::make_shared<nav2_map_server::MapSaver3D>();
      if (map_saver->saveMapTopicToFile(
          map_topic,
          save_parameters.save_parameters_3d))
      {
        retcode = 0;
      } else {
        retcode = 1;
      }

    } else {
      auto map_saver = std::make_shared<nav2_map_server::MapSaver2D>();
      if (map_saver->saveMapTopicToFile(map_topic, save_parameters.save_parameters_2d)) {
        retcode = 0;
      } else {
        retcode = 1;
      }
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
    retcode = -1;
  }

  // Exit
  rclcpp::shutdown();
  return retcode;
}
