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

#ifndef NAV2_ROUTE__CORNER_SMOOTHING_HPP_
#define NAV2_ROUTE__CORNER_SMOOTHING_HPP_

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
  CornerArc(
    const Coordinates & start, const Coordinates & corner, const Coordinates & end,
    float minimum_radius)
  {
    start_edge_length_ = hypotf(corner.x - start.x, corner.y - start.y);
    end_edge_length_ = hypotf(end.x - corner.x, end.y - corner.y);

    // invalid scenario would cause equations to blow up
    if(start_edge_length_ == 0.0 || end_edge_length_ == 0.0){ return; }

    double angle = getAngleBetweenEdges(start, corner, end);

    double tangent_length = minimum_radius / (std::tan(std::fabs(angle) / 2));

    if (tangent_length < start_edge_length_ && tangent_length < end_edge_length_) {
      std::vector<double> start_edge_unit_tangent =
      {(start.x - corner.x) / start_edge_length_, (start.y - corner.y) / start_edge_length_};
      std::vector<double> end_edge_unit_tangent =
      {(end.x - corner.x) / end_edge_length_, (end.y - corner.y) / end_edge_length_};

      double bisector_x = start_edge_unit_tangent[0] + end_edge_unit_tangent[0];
      double bisector_y = start_edge_unit_tangent[1] + end_edge_unit_tangent[1];
      double bisector_magnitude = std::sqrt(bisector_x * bisector_x + bisector_y * bisector_y);

      std::vector<double> unit_bisector =
      {bisector_x / bisector_magnitude, bisector_y / bisector_magnitude};

      start_coordinate_.x = corner.x + start_edge_unit_tangent[0] * tangent_length;
      start_coordinate_.y = corner.y + start_edge_unit_tangent[1] * tangent_length;

      end_coordinate_.x = corner.x + end_edge_unit_tangent[0] * tangent_length;
      end_coordinate_.y = corner.y + end_edge_unit_tangent[1] * tangent_length;

      double bisector_length = minimum_radius / std::sin(angle / 2);

      circle_center_coordinate_.x = corner.x + unit_bisector[0] * bisector_length;
      circle_center_coordinate_.y = corner.y + unit_bisector[1] * bisector_length;

      valid_corner_ = true;
    }
  }

  ~CornerArc() = default;

  void interpolateArc(
    const float & max_angle_resolution,
    std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    std::vector<double> r_start{start_coordinate_.x - circle_center_coordinate_.x,
      start_coordinate_.y - circle_center_coordinate_.y};
    std::vector<double> r_end{end_coordinate_.x - circle_center_coordinate_.x,
      end_coordinate_.y - circle_center_coordinate_.y};
    double cross = r_start[0] * r_end[1] - r_start[1] * r_end[0];
    double dot = r_start[0] * r_end[0] + r_start[1] * r_end[1];
    double signed_angle = std::atan2(cross, dot);
    //lower limit for N is 1 to protect against divide by 0
    int N = std::max(1, static_cast<int>(std::ceil(std::abs(signed_angle) / max_angle_resolution)));
    double angle_resolution = signed_angle / N;

    float x, y;

    for (int i = 0; i <= N; i++) {
      double angle = i * angle_resolution;
      x = circle_center_coordinate_.x +
        (r_start[0] * std::cos(angle) - r_start[1] * std::sin(angle));
      y = circle_center_coordinate_.y +
        (r_start[0] * std::sin(angle) + r_start[1] * std::cos(angle));
      poses.push_back(utils::toMsg(x, y));
    }
  }


  bool isCornerValid() const {return valid_corner_;}

  Coordinates getCornerStart() const {return start_coordinate_;}

  Coordinates getCornerEnd() const {return end_coordinate_;}

protected:
  double getAngleBetweenEdges(
    const Coordinates & start, const Coordinates & corner,
    const Coordinates & end)
  {
    double start_dx = start.x - corner.x;
    double start_dy = start.y - corner.y;

    double end_dx = end.x - corner.x;
    double end_dy = end.y - corner.y;

    double angle =
      acos((start_dx * end_dx + start_dy * end_dy) / (start_edge_length_ * end_edge_length_));

    return angle;
  }

private:
  bool valid_corner_{false};
  float start_edge_length_;
  float end_edge_length_;
  Coordinates start_coordinate_;
  Coordinates end_coordinate_;
  Coordinates circle_center_coordinate_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__CORNER_SMOOTHING_HPP_
