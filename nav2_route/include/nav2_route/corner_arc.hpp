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
  CornerArc(EdgePtr start_edge, EdgePtr end_edge, float minimum_radius)
  {
    double angle = getAngleBetweenEdges(start_edge, end_edge);

    double tangent_length = minimum_radius/(std::tan(angle/2));

    float start_edge_length = start_edge->getEdgeLength();
    float end_edge_length = end_edge->getEdgeLength();

    if(tangent_length < start_edge_length && tangent_length < end_edge_length){
      std::vector<double> start_edge_unit_tangent;
      std::vector<double> end_edge_unit_tangent;
      std::vector<double> unit_bisector;

      start_edge_unit_tangent.push_back((start_edge->start->coords.x -start_edge->end->coords.x)/start_edge_length);
      start_edge_unit_tangent.push_back((start_edge->start->coords.y -start_edge->end->coords.y)/start_edge_length);
      end_edge_unit_tangent.push_back((end_edge->end->coords.x - end_edge->start->coords.x)/end_edge_length);
      end_edge_unit_tangent.push_back((end_edge->end->coords.y - end_edge->start->coords.y)/end_edge_length);

      double bisector_x = start_edge_unit_tangent[0]+end_edge_unit_tangent[0];
      double bisector_y = start_edge_unit_tangent[1]+end_edge_unit_tangent[1];
      double bisector_magnitude = std::sqrt(bisector_x*bisector_x + bisector_y*bisector_y);

      unit_bisector.push_back(bisector_x/bisector_magnitude);
      unit_bisector.push_back(bisector_y/bisector_magnitude);

      start_coordinate_.x = start_edge->end->coords.x + start_edge_unit_tangent[0]*tangent_length;
      start_coordinate_.y = start_edge->end->coords.y + start_edge_unit_tangent[1]*tangent_length;

      end_coordinate_.x = end_edge->start->coords.x + end_edge_unit_tangent[0]*tangent_length;
      end_coordinate_.y = end_edge->start->coords.y + end_edge_unit_tangent[1]*tangent_length;

      double signed_angle = getSignedAngleBetweenEdges(start_edge, end_edge);

      double bisector_length = minimum_radius/std::sin(signed_angle/2);

      circle_center_coordinate_.x = end_edge->start->coords.x + unit_bisector[0]*bisector_length;
      circle_center_coordinate_.y = end_edge->start->coords.y + unit_bisector[1]*bisector_length;

      valid_corner_ = true;

    }
  }

  ~CornerArc() = default;

  void interpolateArc(const float & max_angle_resolution, std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {

    std::vector<double> r_start{ start_coordinate_.x-circle_center_coordinate_.x, start_coordinate_.y-circle_center_coordinate_.y };
    std::vector<double> r_end{ end_coordinate_.x-circle_center_coordinate_.x,  end_coordinate_.y-circle_center_coordinate_.y };
    double cross = r_start[0]*r_end[1] - r_start[1]*r_end[0];
    double dot = r_start[0]*r_end[0] + r_start[1]*r_end[1];
    double signed_angle = std::atan2(cross, dot);
    int N = std::ceil(std::abs(signed_angle)/max_angle_resolution);
    double angle_resolution = signed_angle/N;

    float x, y;

    for(int i = 0; i <= N; i++){
      double angle = i*angle_resolution;
      x = circle_center_coordinate_.x + (r_start[0]*std::cos(angle) - r_start[1]*std::sin(angle));
      y = circle_center_coordinate_.y + (r_start[0]*std::sin(angle) + r_start[1]*std::cos(angle));
      poses.push_back(utils::toMsg(x, y));
    }
  }


  bool isCornerValid() const { return valid_corner_; }

  Coordinates getCornerStart() const { return start_coordinate_; }

  Coordinates getCornerEnd() const { return end_coordinate_; }

protected:
  double getAngleBetweenEdges(const EdgePtr start_edge, const EdgePtr end_edge){

    double start_dx = start_edge->start->coords.x - start_edge->end->coords.x;
    double start_dy = start_edge->start->coords.y - start_edge->end->coords.y;

    double end_dx = end_edge->end->coords.x - end_edge->start->coords.x;
    double end_dy = end_edge->end->coords.y - end_edge->start->coords.y;

    double angle = acos((start_dx*end_dx + start_dy*end_dy)/(start_edge->getEdgeLength()*end_edge->getEdgeLength()));

    return angle;
  }

  double getSignedAngleBetweenEdges(const EdgePtr start_edge, const EdgePtr end_edge){

    double start_edge_length = start_edge->getEdgeLength();
    double end_edge_length = end_edge->getEdgeLength();

    double start_dx = (start_edge->start->coords.x - start_edge->end->coords.x)/start_edge_length;
    double start_dy = (start_edge->start->coords.y - start_edge->end->coords.y)/start_edge_length;

    double end_dx = (end_edge->end->coords.x - end_edge->start->coords.x)/end_edge_length;
    double end_dy = (end_edge->end->coords.y - end_edge->start->coords.y)/end_edge_length;

    double dot = start_dx*end_dx + start_dy*end_dy;
    dot = std::clamp(dot, -1.0, 1.0);

    double angle = std::acos(dot);

    return angle;
  }

private:
  bool valid_corner_{false};
  Coordinates start_coordinate_;
  Coordinates end_coordinate_;
  Coordinates circle_center_coordinate_;
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__CORNER_ARC_HPP_
