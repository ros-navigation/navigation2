// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA

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
  explicit ParticleFilter(const int &initial_number_of_particles);
  void update();
  void resample();
  // TODO(unassigned): Type? get_most_likely_pose();

private:
  std::vector<particle> particles_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
