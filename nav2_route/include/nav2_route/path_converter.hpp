// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROUTE__PATH_CONVERTER_HPP_
#define NAV2_ROUTE__PATH_CONVERTER_HPP_

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"
#include "nav2_route/corner_smoothing.hpp"

namespace nav2_route
{
/**
 * @class nav2_route::PathConverter
 * @brief An helper to convert the route into dense paths
 */
class PathConverter
{
public:
  /**
   * @brief A constructor for nav2_route::PathConverter
   */
  PathConverter() = default;

  /**
   * @brief A destructor for nav2_route::PathConverter
   */
  ~PathConverter() = default;

  /**
   * @brief Configure the object
   * @param node Node to use to get params and create interfaces
   */
  void configure(nav2_util::LifecycleNode::SharedPtr node);

  /**
   * @brief Convert a Route into a dense path
   * @param route Route object
   * @param rerouting_info Re-Route info in case partial path to be populated
   * @param frame Frame ID from planning
   * @param now Time
   * @return Path of the route
   */
  nav_msgs::msg::Path densify(
    const Route & route,
    const ReroutingState & rerouting_info,
    const std::string & frame,
    const rclcpp::Time & now);

  /**
   * @brief Convert an individual edge into a dense line
   * @param x0 X initial
   * @param y0 Y initial
   * @param x1 X final
   * @param y1 Y final
   * @param poses Pose vector reference to populate
   */
  void interpolateEdge(
    float x0, float y0, float x1, float y1,
    std::vector<geometry_msgs::msg::PoseStamped> & poses);

protected:
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("PathConverter")};
  float density_;
  float smoothing_radius_;
  bool smooth_corners_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__PATH_CONVERTER_HPP_
