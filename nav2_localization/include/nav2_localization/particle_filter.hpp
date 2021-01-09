#ifndef NAV2_LOCALIZATION__DUMMY_PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__DUMMY_PARTICLE_FILTER_HPP_

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
	// TODO: Pass initial pose
	ParticleFilter(const int &initial_number_of_particles);
	void update();
	void resample();
	// TODO
	// Type? get_most_likely_pose();

private:
	std::vector<particle> particles_;
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__DUMMY_PARTICLE_FILTER_HPP_
