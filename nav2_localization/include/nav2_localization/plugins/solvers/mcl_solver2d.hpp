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

#ifndef NAV2_LOCALIZATION__PLUGINS__SOLVERS__MCL_SOLVER2D_HPP_
#define NAV2_LOCALIZATION__PLUGINS__SOLVERS__MCL_SOLVER2D_HPP_

#include <memory>  // For shared_ptr<>

// Interfaces
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Others
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_localization/particle_filter.hpp"

namespace nav2_localization
{
class MCLSolver2d : public nav2_localization::Solver
{
public:
  MCLSolver2d() {}

  geometry_msgs::msg::TransformStamped solve(
    const geometry_msgs::msg::TransformStamped & curr_odom,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) override;

  void initFilter(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & pose) override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    SampleMotionModel::Ptr & motionSampler,
    Matcher2d::Ptr & matcher,
    const geometry_msgs::msg::TransformStamped & odom,
    const geometry_msgs::msg::TransformStamped & pose) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

private:
  std::shared_ptr<ParticleFilter> pf_;  // Particle filter
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PLUGINS__SOLVERS__MCL_SOLVER2D_HPP_
