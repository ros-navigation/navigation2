#ifndef NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_

#include <filter/bootstrapfilter.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace nav2_localization
{
class CustomParticleFilter : public BFL::BootstrapFilter<geometry_msgs::msg::TransformStamped, sensor_msgs::msg::LaserScan>
{
public:
	CustomParticleFilter(
		BFL::MCPdf<geometry_msgs::msg::TransformStamped> * prior,
		int resampleperiod = 0,
		double resamplethreshold = 0,
		int resamplescheme = DEFAULT_RS);

	std::vector<BFL::WeightedSample<geometry_msgs::msg::TransformStamped>> getNewSamples();

};
} // nav2_localization

#endif // NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
