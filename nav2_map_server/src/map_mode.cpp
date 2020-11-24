// Copyright 2019 Rover Robotics
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

#include "nav2_map_server/map_mode.hpp"

#include <stdexcept>
#include <string>

namespace nav2_map_server
{
const char * map_mode_to_string(MapMode map_mode)
{
  switch (map_mode) {
    case MapMode::Trinary:
      return "trinary";
    case MapMode::Scale:
      return "scale";
    case MapMode::Raw:
      return "raw";
    default:
      throw std::invalid_argument("map_mode");
  }
}

MapMode map_mode_from_string(std::string map_mode_name)
{
  for (auto & c : map_mode_name) {
    c = tolower(c);
  }

  if (map_mode_name == "scale") {
    return MapMode::Scale;
  } else if (map_mode_name == "raw") {
    return MapMode::Raw;
  } else if (map_mode_name == "trinary") {
    return MapMode::Trinary;
  } else {
    throw std::invalid_argument("map_mode_name");
  }
}
}  // namespace nav2_map_server
