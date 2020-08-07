// Copyright (c) 2020 Shivam Pandey pandeyshivam2017robotics@gmail.com
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
//
// Created by shivam on 7/23/20.
//

#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "nav2_map_server/map_saver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server_3d/map_io_3d.hpp"

using namespace nav2_map_server;  // NOLINT

const char * USAGE_STRING{
  "Usage:\n"
  "  map_saver_cli_3D [arguments] [--ros-args ROS remapping args]\n"
  "\n"
  "Arguments:\n"
  "  -h/--help\n"
  "  -t <map_topic>\n"
  "  -f <mapname>\n"
  "  --fmt <file_format>\n"
  "  --as_bin Give the flag to save map with binary encodings\n"
  "  --origin-orientation <[size 7 vector of floats]>"
  "\n"
  "NOTE: --ros-args should be passed at the end of command line"};

typedef enum
{
  COMMAND_MAP_TOPIC,
  COMMAND_MAP_FILE_NAME,
  COMMAND_FILE_FORMAT,
  COMMAND_ENCODING,
  COMMAND_VIEW_POINT
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
  std::string & map_topic, nav2_map_server_3d::SaveParameters & save_parameters)
{
  const struct cmd_struct commands[] = {
    {"-t", COMMAND_MAP_TOPIC},
    {"-f", COMMAND_MAP_FILE_NAME},
    {"--fmt", COMMAND_FILE_FORMAT},
    {"--as_bin", COMMAND_ENCODING},
    {"--origin-orientation", COMMAND_VIEW_POINT}
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
        if ((it + 1) == arguments.end() && *it != "--as_bin") {
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
          case COMMAND_FILE_FORMAT:
            save_parameters.format = *it;
            break;
          case COMMAND_ENCODING:
            it--;  // as this one is a simple flag that puts binary format to on
            save_parameters.as_binary = true;
            break;
          case COMMAND_VIEW_POINT:
            for (int k = 0; k < 9; ++k) {
              std::string tmp = *it;
              if (tmp == "[") {
                it++;
                continue;
              } else if (tmp == "]") {
                break;
              }
              if (k < 4) {
                save_parameters.origin.push_back(std::stof(tmp.substr(0, tmp.size() - 1)));
              } else if (k < 7) {
                save_parameters.orientation.push_back(std::stof(tmp.substr(0, tmp.size() - 1)));
              } else if (k == 7) {
                save_parameters.orientation.push_back(std::stof(tmp));
              }
              it++;
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
  auto logger = rclcpp::get_logger("map_saver_cli_3D");

  // Parse CLI-arguments
  nav2_map_server_3d::SaveParameters save_parameters;
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
