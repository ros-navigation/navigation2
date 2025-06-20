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
#include <algorithm>

#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"

namespace nav2_route
{
/**
 * @class nav2_route::CornerArc
 * @brief A class used to smooth corners defined by the edges and nodes
 * of the route graph. Used with path converter to output a corner smoothed plan
 */
class CornerArc
{
public:
  /**
   * @brief A constructor for nav2_route::CornerArc
   * @param start start coordinate of corner to be smoothed
   * @param corner corner coordinate of corner to be smoothed
   * @param end end coordinate of corner to be smoothed
   * @param minimum_radius smoothing radius to fit to the corner
   */
  CornerArc(
    const Coordinates & start, const Coordinates & corner, const Coordinates & end,
    float minimum_radius)
  {
    start_edge_length_ = hypotf(corner.x - start.x, corner.y - start.y);
    end_edge_length_ = hypotf(end.x - corner.x, end.y - corner.y);

    // invalid scenario would cause equations to blow up
    if (start_edge_length_ == 0.0 || end_edge_length_ == 0.0) {return;}

    float angle = getAngleBetweenEdges(start, corner, end);

    // cannot smooth a 0 (back and forth) or 180 (straight line) angle
    if (std::abs(angle) < 1E-6 || std::abs(angle - M_PI) < 1E-6) {return;}

    float tangent_length = minimum_radius / (std::tan(std::fabs(angle) / 2));

    if (tangent_length < start_edge_length_ && tangent_length < end_edge_length_) {
      std::vector<float> start_edge_unit_tangent =
      {(start.x - corner.x) / start_edge_length_, (start.y - corner.y) / start_edge_length_};
      std::vector<float> end_edge_unit_tangent =
      {(end.x - corner.x) / end_edge_length_, (end.y - corner.y) / end_edge_length_};

      float bisector_x = start_edge_unit_tangent[0] + end_edge_unit_tangent[0];
      float bisector_y = start_edge_unit_tangent[1] + end_edge_unit_tangent[1];
      float bisector_magnitude = std::sqrt(bisector_x * bisector_x + bisector_y * bisector_y);

      std::vector<float> unit_bisector =
      {bisector_x / bisector_magnitude, bisector_y / bisector_magnitude};

      start_coordinate_.x = corner.x + start_edge_unit_tangent[0] * tangent_length;
      start_coordinate_.y = corner.y + start_edge_unit_tangent[1] * tangent_length;

      end_coordinate_.x = corner.x + end_edge_unit_tangent[0] * tangent_length;
      end_coordinate_.y = corner.y + end_edge_unit_tangent[1] * tangent_length;

      float bisector_length = minimum_radius / std::sin(angle / 2);

      circle_center_coordinate_.x = corner.x + unit_bisector[0] * bisector_length;
      circle_center_coordinate_.y = corner.y + unit_bisector[1] * bisector_length;

      valid_corner_ = true;
    }
  }

  /**
   * @brief A destructor for nav2_route::CornerArc
   */
  ~CornerArc() = default;

  /**
   * @brief interpolates the arc for a path of certain density
   * @param max_angle_resolution Resolution to interpolate path to
   * @param poses Pose output
   */
  void interpolateArc(
    const float & max_angle_resolution,
    std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    std::vector<float> r_start{start_coordinate_.x - circle_center_coordinate_.x,
      start_coordinate_.y - circle_center_coordinate_.y};
    std::vector<float> r_end{end_coordinate_.x - circle_center_coordinate_.x,
      end_coordinate_.y - circle_center_coordinate_.y};
    float cross = r_start[0] * r_end[1] - r_start[1] * r_end[0];
    float dot = r_start[0] * r_end[0] + r_start[1] * r_end[1];
    float signed_angle = std::atan2(cross, dot);
    // lower limit for N is 1 to protect against divide by 0
    int N = std::max(1, static_cast<int>(std::ceil(std::abs(signed_angle) / max_angle_resolution)));
    float angle_resolution = signed_angle / N;

    float x, y;
    for (int i = 0; i <= N; i++) {
      float angle = i * angle_resolution;
      x = circle_center_coordinate_.x +
        (r_start[0] * std::cos(angle) - r_start[1] * std::sin(angle));
      y = circle_center_coordinate_.y +
        (r_start[0] * std::sin(angle) + r_start[1] * std::cos(angle));
      poses.push_back(utils::toMsg(x, y));
    }
  }

  /**
   * @brief return if a valid corner arc (one that doesn't overrun the edge lengths) is generated
   * @return if the a corner was able to be generated
   */
  bool isCornerValid() const {return valid_corner_;}

  /**
   * @brief return the start coordinate of the corner arc
   * @return start coordinate of the arc
   */
  Coordinates getCornerStart() const {return start_coordinate_;}

  /**
   * @brief return the end coordinate of the corner arc
   * @return end coordinate of the arc
   */
  Coordinates getCornerEnd() const {return end_coordinate_;}

protected:
  /**
   * @brief find the signed angle between a corner generated by 3 points
   * @param start start coordinate of corner to be smoothed
   * @param corner corner coordinate of corner to be smoothed
   * @param end end coordinate of corner to be smoothed
   * @return signed angle of the corner
   */
  float getAngleBetweenEdges(
    const Coordinates & start, const Coordinates & corner,
    const Coordinates & end)
  {
    float start_dx = start.x - corner.x;
    float start_dy = start.y - corner.y;

    float end_dx = end.x - corner.x;
    float end_dy = end.y - corner.y;

    float angle =
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
