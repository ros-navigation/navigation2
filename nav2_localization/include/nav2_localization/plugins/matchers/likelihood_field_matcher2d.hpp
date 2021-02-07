// Copyright (c) 2021 Jose M. TORRES-CAMARA and Khaled SAAD
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
// limitations under the License. Reserved.

#ifndef NAV2_LOCALIZATION__PLUGINS__MATCHERS__LIKELIHOOD_FIELD_MATCHER2D_HPP_
#define NAV2_LOCALIZATION__PLUGINS__MATCHERS__LIKELIHOOD_FIELD_MATCHER2D_HPP_

#include <vector>  // For vector<>
#include <unordered_map>
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
class LikelihoodFieldMatcher2d : public Matcher2d
{
public:
  LikelihoodFieldMatcher2d()
  : Matcher2d() {}

  double getScanProbability(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan,
    const geometry_msgs::msg::TransformStamped & curr_pose) override;
  void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr & map) override;
  void setSensorPose(const geometry_msgs::msg::TransformStamped & sensor_pose) override;
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

private:
  /**
   * @brief Computes and caches the likelihood field for the set map.
   */
  void preComputeLikelihoodField();

  /**
   * @brief Depth-First Search (DFS). This is used within the preComputeLikelihoodField method to speed up the computation
   * @param index_curr Index of current map cell
   * @param index_of_obstacle Index of an obstacle
   * @param visited A vector of all the cells that have been visited so far
   */
  void DFS(const int & index_curr, const int & index_of_obstacle, std::vector<bool> & visited);

  std::unordered_map<int, double> pre_computed_likelihood_field_;  // Cached likelihood values
  double max_likelihood_distace_;  // The distance beyond which the likelihood is 0
  double max_likelihood_dist_prob_;
  int max_number_of_beams_;  // The maximum number of beams to use when matching

  // Matcher parameters
  double sigma_hit_;
  double z_hit_;
  double z_rand_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PLUGINS__MATCHERS__LIKELIHOOD_FIELD_MATCHER2D_HPP_
