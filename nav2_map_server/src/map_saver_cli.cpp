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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("map_saver_cli");

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h" || *it == "--help") {
      std::cout << USAGE_STRING << std::endl;
      return 0;
    } else if (*it == "-f") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: -f should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("output_file_no_ext", *it);
    } else if (*it == "--occ") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --occ should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("threshold_occupied", atoi(it->c_str()));
    } else if (*it == "--free") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --free should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("threshold_free", atoi(it->c_str()));
    } else if (*it == "--mode") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --mode should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("map_mode", *it);
    } else if (*it == "--fmt") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --fmt should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("image_format", *it);
    } else {
      RCLCPP_WARN(logger, "Ignoring unrecognized argument '%s'", it->c_str());
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
