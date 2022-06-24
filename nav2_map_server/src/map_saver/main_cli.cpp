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

#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace nav2_map_server;  // NOLINT

const char * USAGE_STRING{
  "Usage:\n"
  "  map_saver_cli [arguments] [--ros-args ROS remapping args]\n"
  "\n"
  "Arguments:\n"
  "  -h/--help\n"
  "  -t <map_topic>\n"
  "  -f <mapname>\n"
  "  --occ <threshold_occupied>\n"
  "  --free <threshold_free>\n"
  "  --fmt <image_format>\n"
  "  --mode trinary(default)/scale/raw\n"
  "\n"
  "NOTE: --ros-args should be passed at the end of command line"};

typedef enum
{
  COMMAND_MAP_TOPIC,
  COMMAND_MAP_FILE_NAME,
  COMMAND_IMAGE_FORMAT,
  COMMAND_OCCUPIED_THRESH,
  COMMAND_FREE_THRESH,
  COMMAND_MODE
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

// Arguments parser
// Input parameters: logger, argc, argv
// Output parameters: map_topic, save_parameters
ARGUMENTS_STATUS parse_arguments(
  const rclcpp::Logger & logger, int argc, char ** argv,
  std::string & map_topic, SaveParameters & save_parameters)
{
  const struct cmd_struct commands[] = {
    {"-t", COMMAND_MAP_TOPIC},
    {"-f", COMMAND_MAP_FILE_NAME},
    {"--occ", COMMAND_OCCUPIED_THRESH},
    {"--free", COMMAND_FREE_THRESH},
    {"--mode", COMMAND_MODE},
    {"--fmt", COMMAND_IMAGE_FORMAT},
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
            save_parameters.map_file_name = *it;
            break;
          case COMMAND_FREE_THRESH:
            save_parameters.free_thresh = atof(it->c_str());
            break;
          case COMMAND_OCCUPIED_THRESH:
            save_parameters.occupied_thresh = atof(it->c_str());
            break;
          case COMMAND_IMAGE_FORMAT:
            save_parameters.image_format = *it;
            break;
          case COMMAND_MODE:
            try {
              save_parameters.mode = map_mode_from_string(*it);
            } catch (std::invalid_argument &) {
              save_parameters.mode = MapMode::Trinary;
              RCLCPP_WARN(
                logger,
                "Map mode parameter not recognized: %s, using default value (trinary)",
                it->c_str());
            }
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
  SaveParameters save_parameters;
  std::string map_topic = "map";
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
    auto map_saver = std::make_shared<nav2_map_server::MapSaver>();
    map_saver->on_configure(rclcpp_lifecycle::State());
    if (map_saver->saveMapTopicToFile(map_topic, save_parameters)) {
      retcode = 0;
    } else {
      retcode = 1;
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
    retcode = -1;
  }

  // Exit
  rclcpp::shutdown();
  return retcode;
}
