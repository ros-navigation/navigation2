// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_UTIL__STRING_UTILS_HPP_
#define NAV2_UTIL__STRING_UTILS_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace nav2_util
{

typedef std::vector<std::string> Tokens;

/*
 * @brief Remove leading slash from a topic name
 * @param in String of topic in
 * @return String out without slash
*/
std::string strip_leading_slash(const std::string & in);

///
/*
 * @brief Split a string at the delimiters
 * @param in String to split
 * @param Delimiter criteria
 * @return Tokens
*/
Tokens split(const std::string & tokenstring, char delimiter);


/** @brief Parse a vector of vector of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return)

// function to convert polygon in vector of points
std::vector<geometry_msgs::msg::Point> 
    toPointVector(geometry_msgs::msg::Polygon::SharedPtr polygon);


std::vector<geometry_msgs::msg::Point> 
  makeVectorPointsFromString(const std::string & safety_zone_str,
  std::vector<geometry_msgs::msg::Point> & safety_zone); 


  
    }


}  // namespace nav2_util

#endif  // NAV2_UTIL__STRING_UTILS_HPP_
