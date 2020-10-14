// Copyright (c) 2020 Fetullah Atas
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

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__COMMON_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__COMMON_HPP_

#include <string>

#include "opencv2/opencv.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nav2_waypoint_follower
{

/**
 * @brief given encoding string , determine corresponding CV format
 *
 * @param encoding
 * @return int
 */
int encoding2mat_type(const std::string & encoding);

/**
 * @brief given CV type encoding return corresponding sensor_msgs::msg::Image::Encoding string
 *
 * @param mat_type
 * @return std::string
 */
std::string mat_type2encoding(int mat_type);

}  // namespace nav2_waypoint_follower
#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__COMMON_HPP_
