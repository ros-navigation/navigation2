// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
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

#ifndef NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_

#include <vector>

namespace nav2_localization
{
struct particle
{
  // pose
  double weight;
};

class ParticleFilter
{
public:
  // TODO(unassigned): Pass initial pose
  explicit ParticleFilter(const int & initial_number_of_particles);
  void update();
  void resample();
  // TODO(unassigned): Type? get_most_likely_pose();

private:
  std::vector<particle> particles_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
