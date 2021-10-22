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


#ifndef NAV2_LOCALIZATION__PLUGINS__PARTICLE_FILTERS__MCL_HPP_
#define NAV2_LOCALIZATION__PLUGINS__PARTICLE_FILTERS__MCL_HPP_

#include "nav2_localization/interfaces/particle_filter_base.hpp"

namespace nav2_localization
{
class MCL : public nav2_localization::ParticleFilter
{
public:
    MCL();
    nav_msgs::msg::Odometry estimatePose(
        const nav_msgs::msg::Odometry & curr_odom,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) override;

protected:
    /**
    * @brief
    * Resamples and updates the particle set using low variance resampling
    */

    void lowVarianceResample();

    /**
    * @brief
    * Calculates the pose mean and covariance from the particle set
    */
    nav_msgs::msg::Odometry getMeanPose();
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PLUGINS__PARTICLE_FILTERS__MCL_HPP_
