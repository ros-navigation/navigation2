// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
#include <limits>

#include "nav2_planner/path_validator.hpp"
#include "nav2_util/geometry_utils.hpp"


namespace nav2_planner
{

bool PathValidator::check(const Path &, float, bool)
{
  return true;
}

PathIndex PathValidator::truncate(const Path & path, const Pose & current_pose)
{
  PathIndex closest_point_index = 0;
  float current_distance = std::numeric_limits<float>::max();
  float closest_distance = current_distance;
  geometry_msgs::msg::Point current_point = current_pose.pose.position;
  for (unsigned int i = 0; i != path.poses.size(); ++i) {
    Point path_point = path.poses[i].pose.position;

    current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

    if (current_distance < closest_distance) {
      closest_point_index = i;
      closest_distance = current_distance;
    }
  }
  return closest_point_index;
}
bool PathValidator::isCostChangeSignifant(
  const Costs & original_costs,
  const Costs & current_costs,
  float z_score)
{
  if (original_costs.size() != current_costs.size()) {
    return true;
  }

  float original_costs_sum = std::accumulate(original_costs.begin(), original_costs.end(), 0.0f);
  float original_costs_mean = original_costs_sum / static_cast<float>(original_costs.size());

  std::cout << "Original Mean: " << original_costs_mean << std::endl;
  float current_costs_sum = std::accumulate(current_costs.begin(), current_costs.end(), 0.0f);
  float current_costs_mean = current_costs_sum / static_cast<float>(current_costs.size());

  std::cout << "Current Mean: " << current_costs_mean << std::endl;
  float variance_original = 0;
  for (const auto & cost : original_costs) {
    variance_original += (cost - original_costs_mean) * (cost - original_costs_mean);
  }
  variance_original /= static_cast<float>(original_costs.size() - 1);


  float variance_current = 0;
  for (const auto & cost : current_costs) {
    variance_current += (cost - current_costs_mean) * (cost - current_costs_mean);
  }
  variance_current /= static_cast<float>(current_costs.size() - 1);

  // Conduct a two sample Z-test, with the null hypothesis is that both path cost samples
  // come from the same distribution (e.g. there is not a statistically significant change)
  // Thus, the difference in population mean is 0 and the sample sizes are the same
  float numerator = current_costs_mean - original_costs_mean;
  float denominator = std::sqrt((variance_current + variance_original) /
      static_cast<float>(current_costs.size()));

  // if the variance of the current and original costs are 0
  if (denominator <= 0) {
    if ( std::fabs(current_costs_mean - original_costs_mean) > 10) {
      return true;
    } else {
      return false;
    }
  }

  float cost_change_z_score = numerator / denominator;

  std::cout << "Cost Change z score: " << cost_change_z_score << std::endl;
  if (cost_change_z_score > z_score) {
    return false;
  }
  return true;
}

bool isThisALethal(const Path)
{
  return true;
}
}
