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
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_localization
{

class Particle
{
public:
  Particle(const geometry_msgs::msg::TransformStamped &inital_pose, const double &initial_weight) : pose_(inital_pose), weight_(initial_weight) {};
  geometry_msgs::msg::TransformStamped pose_;
  double weight_;
};

class ParticleFilter
{
public:
  using Ptr = std::shared_ptr<nav2_localization::ParticleFilter>;
  explicit ParticleFilter(const int & initial_number_of_particles);
  virtual geometry_msgs::msg::TransformStamped estimatePose() = 0;

private:
  std::vector<Particle> particles_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
