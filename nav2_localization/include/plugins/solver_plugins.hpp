#ifndef NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_

#include <interfaces/solver_base.hpp>
#include <map.hpp>
#include <particle_filter.hpp>

namespace nav2_localization_plugins
{
	// Implements a 2D Monte-Carlo Localization (MCL) algorithm to solve the state estimation problem.
	class MCL2d : public nav2_localization_base::Solver
	{
		private:
			nav2_localization::ParticleFilter2d particle_filter; // MCL solver, a Particle Filter (PF)

		public:
			MCL2d(){} // Class constructor. Empty bc the PF will be initialized in the "sample()" step
			
			// Methods implementing the MCL steps:
			// TODO - How to send map?
			void sample(const nav2_localization::ndt_grid_2d_t& map, const unsigned int& n_particles); // Sample the map with the specified number of particles 
			// TODO - How to send odometry?
			void update(const Odometry& odom, const nav2_localization_base::MotionModel& motion_model); // Update the particles' position using a given odometry and motion model
			// TODO - How to send map?
			void resample(const nav2_localization::ndt_grid_2d_t& map); // Compute particles' weights, decide which to kill and generate new samples
	};
};

#endif // NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
