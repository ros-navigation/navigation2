#ifndef NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_

#include <filter/bootstrapfilter.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_localization
{
class CustomParticleFilter : public BFL::BootstrapFilter<geometry_msgs::msg::TransformStamped, double>
{
public:
	CustomParticleFilter(
		BFL::MCPdf<geometry_msgs::msg::TransformStamped> * prior,
		int resampleperiod = 0,
		double resamplethreshold = 0,
		int resamplescheme = DEFAULT_RS);

	vector<WeightedSample<geometry_msgs::msg::TransformStamped>> getNewSamples();

};
} // nav2_localization

#endif // NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
