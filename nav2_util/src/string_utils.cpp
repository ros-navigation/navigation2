// Copyright (c) 2019 Intel Corporation
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

#include "nav2_util/string_utils.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>

using std::string;

namespace nav2_util
{

std::string strip_leading_slash(const string & in)
{
  string out = in;

  if ((!in.empty()) && (in[0] == '/')) {
    out.erase(0, 1);
  }

  return out;
}

Tokens split(const string & tokenstring, char delimiter)
{
  Tokens tokens;

  size_t current_pos = 0;
  size_t pos = 0;
  while ((pos = tokenstring.find(delimiter, current_pos)) != string::npos) {
    tokens.push_back(tokenstring.substr(current_pos, pos - current_pos));
    current_pos = pos + 1;
  }
  tokens.push_back(tokenstring.substr(current_pos));
  return tokens;
}


/** @brief Parse a vector of vector of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return)
{
  std::vector<std::vector<float>> result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof()) {
    switch (input_ss.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
        float value;
        input_ss >> value;
        if (!!input_ss) {
          current_vector.push_back(value);
        }
        break;
    }
  }

  if (depth != 0) {
    error_return = "Unterminated vector string.";
  } else {
    error_return = "";
  }

  return result;
}



/** @brief Parse a vector of vector of floats from a string & convert to vector of points. */
std::vector<geometry_msgs::msg::Point> makeVectorPointsFromString(
    const std::string & safety_zone_str,
    std::vector<geometry_msgs::msg::Point> & safety_zone)
{
  std::string error;
  std::vector<std::vector<float>> vvf = parseVVF(safety_zone_str, error);
  if (error != "") {
        RCLCPP_ERROR(
        logger_, "Error parsing safety_zone : '%s'", error.c_str());
        RCLCPP_ERROR(
        logger_, "  Safety_zone string was '%s'.", safety_zone_str.c_str());
    }
  // convert vvf into points.
  if (vvf.size() < 3) {
      RCLCPP_ERROR(
      logger_,
      "You must specify at least three points for the robot safety_zone, reverting to previous safety_zone."); //NOLINT
  }
  safety_zone.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++) {
      if (vvf[i].size() == 2) {
      geometry_msgs::msg::Point point;
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      point.z = 0;
      safety_zone.push_back(point);
      } else {
      RCLCPP_ERROR(
          logger_,
          "Points in the safety_zone specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
          static_cast<int>(vvf[i].size()));
      }
  }
  return safety_zone;
}


}  // namespace nav2_util
