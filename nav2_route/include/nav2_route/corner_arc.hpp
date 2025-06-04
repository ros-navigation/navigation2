// Copyright (c) 2025, Polymath Robotics
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

#ifndef NAV2_ROUTE__CORNER_ARC_HPP_
#define NAV2_ROUTE__CORNER_ARC_HPP_

#include <vector>
#include <cmath>

#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"

namespace nav2_route
{
class CornerArc
{
public:
  /**
   * @brief Constructor
   */
  CornerArc(EdgePtr start_edge, EdgePtr end_edge, double minimum_radius);

  ~CornerArc() = default;

  void interpolateArc(const float & max_angle_resolution, std::vector<geometry_msgs::msg::PoseStamped> & poses);

  double getAngleBetweenEdges(const EdgePtr start_edge, const EdgePtr end_edge);

  double getSignedAngleBetweenEdges(const EdgePtr start_edge, const EdgePtr end_edge);

  bool isCornerValid() const { return valid_corner_; }

  Coordinates getCornerStart() const { return start_coordinate_; }

  Coordinates getCornerEnd() const { return end_coordinate_; }

private:
  EdgePtr start_edge_;
  EdgePtr end_edge_;
  float start_edge_length_;
  float end_edge_length_;
  double minimum_radius_;
  double signed_angle_;
  bool valid_corner_{false};
  Coordinates start_coordinate_;
  Coordinates end_coordinate_;
  Coordinates circle_center_coordinate_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__CORNER_ARC_HPP_
