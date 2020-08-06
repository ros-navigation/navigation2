#ifndef NAV2_LOCALIZATION__PARTICLE_HPP_
#define NAV2_LOCALIZATION__PARTICLE_HPP_

#include <map.hpp>
#include <measurement.hpp>
#include <vector>

namespace nav2_localization
{
	// Position and orientation for 2D environments
	typedef struct
	{
		// 2D Position (Cartesian coords.)
		float x;
		float y;

		// 2D Orientation (rotation over Z axis)
		float theta;
	} pose_2d_t;

/*
	// Position and orientation for 3D environments
	typedef struct
	{
		// 3D Position
		float x;
		float y;
		float z;

		// 3D Orientation
		float alpha;
		float beta;
		float gamma;
	} pose_3d_t; 
*/


	// Base Particle class
	class Particle:
	{
		protected:
			float weight; // Particle's importance weight

			bool decide_suicide(); // Use the particle's weight to pseudo-randomly decide if the particle should be killed

			virtual void move(); // Move the particle
			virtual void compute_weight(); // Update the weight
	};

	// 2D particle
	class Particle2d : Particle
	{
		private:
			// 2D pose (Cartesian Coords.) and orientation (Z axis)
			pose_2d_t pose;

		public:
			Particle2d(const float& x, const float& y, const float& theta); // Class constructor. Takes initial position (cartesian coords.) and orientation (Z axis)

			void move(const float& dx, const float& dy, const float& dtheta); // Move the particle with the specified variation of position and orientation
			void compute_weight(const ndt_grid_2d_t& map, const measurement_t& measurement); // Update the weight with the given map and observation
	};

/*
	// 3D particle	
	class Particle3d : Particle
	{
		private:
			// 3D pose (Cartesian Coords.) and orientation (Euler Angles)
			pose_3d_t pose;

		public:
			Particle(const float& x, const float& y, const float& z, const float& alpha, const float& beta, const float& gamma); // Class constructor. Takes initial position (cartesian coords.) and orientation (Euler Angles)

			void move(const float& dx, const float& dy, const float& dz, const float& dalpha, const float& dbeta, const float& dgamma); // Move the particle with the specified variation of position and orientation
			void compute_weight(const ndt_grid_3d_t& map, const measurement_t& measurement); // Update the weight with the given map and observation
	}; 
*/
};

#endif // NAV2_LOCALIZATION__PARTICLE_HPP_
