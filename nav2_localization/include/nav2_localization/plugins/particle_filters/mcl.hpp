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

#include "nav2_localization/interfaces/particle_filter_base.hpp"

#ifndef NAV2_LOCALIZATION__PLUGINS__PARTICLE_FILTERS__MCL
#define NAV2_LOCALIZATION__PLUGINS__PARTICLE_FILTERS__MCL

namespace nav2_localization
{
class MCL : public nav2_localization::ParticleFilter
{
public:
    MCL();
    geometry_msgs::msg::TransformStamped estimatePose() override;
};
}   // namesape nav2_localization

#endif // NAV2_LOCALIZATION__PLUGINS__PARTICLE_FILTERS__MCL