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

#ifndef NAV2_LOCALIZATION__INTERFACES__PARTICLE_FILTER_BASE_HPP_
#define NAV2_LOCALIZATION__INTERFACES__PARTICLE_FILTER_BASE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav2_localization/interfaces/solver_base.hpp"

namespace nav2_localization
{

class Particle
{
public:
  Particle(const geometry_msgs::msg::Pose & inital_pose, const double & initial_weight)
  : pose_(inital_pose), weight_(initial_weight) {}
  geometry_msgs::msg::Pose pose_;
  double weight_;
};

class ParticleFilter : public Solver
{
public:
  using Ptr = std::shared_ptr<nav2_localization::ParticleFilter>;
  ParticleFilter();

protected:
  int particles_count_;  // Number of particles
  double particles_spread_radius_;  // Initial particles spread radius
  double particles_spread_yaw_;  // Initial particles yaw spread
  std::vector<Particle> particles_;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    SampleMotionModel::Ptr & motion_sampler,
    Matcher2d::Ptr & matcher) override;

  void initFilter(const geometry_msgs::msg::Pose & pose);

  void initParticles(const geometry_msgs::msg::Pose & inital_pose);
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__INTERFACES__PARTICLE_FILTER_BASE_HPP_
