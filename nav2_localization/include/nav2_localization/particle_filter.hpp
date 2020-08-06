#ifndef NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_

#include <map.hpp>
#include <measurement.hpp>
#include <particle.hpp>
#include <vector>

namespace nav2_localization
{
	// Implements a Particle Filter (PF) for 2D localization purposes
	class ParticleFilter2d
	{
		private:
			std::vector<nav2_localization::Particle2d> particles; // Vector containing all the particles
			unsigned int n_particles; // Number of particles

		public:
			ParticleFilter2d(){}

			void sample(const ndt_grid_2d_t& map, const unsigned int& n_parts); // Takes the map to sample and the number of particles with which the map will be sampled
			void apply_motion(const float& dx, const float& dy, const float& dtheta); // Applies a certain pose variation to every particle of the PF
			void compute_weights(const ndt_grid_2d_t& map, const measurement_t& measurement); // Update the weights of all the particles with the given map and observation
			void purge(); // Kills particles pseudo-randomly according to their importance weights
			// TODO - Update to use KLD Sampling (as AMCL does)
			void resample(); // Generate new particles
	};
};

#endif // NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
