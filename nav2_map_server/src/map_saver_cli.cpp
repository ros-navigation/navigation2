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

#include "nav2_map_server/map_saver.hpp"
#include "rclcpp/rclcpp.hpp"

const char * USAGE_STRING{
  "Usage: \n"
  "  map_saver -h/--help\n"
  "  map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [--fmt <image_format>] "
  "[--mode trinary/scale/raw] [-f <mapname>] [ROS remapping args]"};

typedef enum {TYPE_STR, TYPE_INT} VAR_TYPE;
struct cmd_struct
{
  const char * cmd;
  const char * param_name;
  VAR_TYPE type;
};

int main(int argc, char ** argv)
{
  const struct cmd_struct commands[] = {
    {"-f", "output_file_no_ext", TYPE_STR},
    {"--occ", "threshold_occupied", TYPE_INT},
    {"--free", "threshold_free", TYPE_INT},
    {"--mode", "map_mode", TYPE_STR},
    {"--fmt", "image_format", TYPE_STR},
  };

  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("map_saver_cli");

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;

  size_t cmd_size = sizeof(commands) / sizeof(commands[0]);
  size_t i;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h" || *it == "--help") {
      std::cout << USAGE_STRING << std::endl;
      rclcpp::shutdown();
      return 0;
    }
    for (i = 0; i < cmd_size; i++) {
      if (commands[i].cmd == *it) {
        if ((it + 1) == arguments.end()) {
          RCLCPP_ERROR(logger, "Wrong argument: %s should be followed by a value.", it->c_str());
          rclcpp::shutdown();
          return -1;
        }
        it++;
        if (commands[i].type == TYPE_INT) {
          params_from_args.emplace_back(commands[i].param_name, atoi(it->c_str()));
        } else {
          params_from_args.emplace_back(commands[i].param_name, *it);
        }
        break;
      }
    }
    if (i == cmd_size) {
      RCLCPP_ERROR(logger, "Wrong argument: %s", it->c_str());
      rclcpp::shutdown();
      return -1;
    }
  }

  auto node = std::make_shared<nav2_map_server::MapSaver>(
    rclcpp::NodeOptions().parameter_overrides(params_from_args));
  auto future = node->map_saved_future();

  int retcode;
  switch (rclcpp::spin_until_future_complete(node, future)) {
    case rclcpp::executor::FutureReturnCode::INTERRUPTED:
      std::cout << "Map saver failed: interrupted" << std::endl;
      retcode = 1;
      break;
    case rclcpp::executor::FutureReturnCode::TIMEOUT:
      std::cout << "Map saver failed: timeout" << std::endl;
      retcode = 1;
      break;
    case rclcpp::executor::FutureReturnCode::SUCCESS:
      try {
        future.get();
        std::cout << "Map saver succeeded" << std::endl;
        retcode = 0;
      } catch (std::exception & e) {
        std::cout << "Map saver failed: " << e.what() << std::endl;
        retcode = 1;
      }
      break;
    default:
      std::cerr << "this should never happen" << std::endl;
      retcode = -1;
      break;
  }

  rclcpp::shutdown();
  return retcode;
}
