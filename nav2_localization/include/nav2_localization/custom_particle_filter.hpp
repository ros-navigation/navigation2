#ifndef NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace nav2_localization
{
struct particle
{
	geometry_msgs::msg::TransformStamped pose;
	float weight;
};

class CustomParticleFilter
{
public:
	CustomParticleFilter(
		std::vector<geometry_msgs::msg::TransformStamped> * prior,
		int resampleperiod = 0,
		double resamplethreshold = 0,
		int resamplescheme = DEFAULT_RS);

	std::vector<particle> particles;
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
