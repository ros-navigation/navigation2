// Copyright (c) 2021 Marwan TAHER and Khaled SAAD
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

#ifndef NAV2_LOCALIZATION__PLUGINS__SOLVERS__MCL_SOLVER_HPP_
#define NAV2_LOCALIZATION__PLUGINS__SOLVERS__MCL_SOLVER_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav2_localization/plugins/solvers/particle_filter_base.hpp"

namespace nav2_localization
{

class MCLSolver : public ParticleFilterSolver
{
public:
  MCLSolver();

  /**
  * @brief
  * MCLSolver implementation
  */
  geometry_msgs::msg::PoseWithCovarianceStamped estimatePose(
    const nav_msgs::msg::Odometry & curr_odom,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) override;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PLUGINS__SOLVERS__MCL_SOLVER_HPP_
