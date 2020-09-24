#include "nav2_localization/custom_particle_filter.hpp"

namespace  nav1_localization
{
	CustomParticleFilter::CustomParticleFilter(
		BFL::MCPdf<geometry_msgs::msg::TransformStamped> *prior,
		int resampleperiod,
		double resamplethreshold,
		int resamplescheme):
		BFL::BootstrapFilter<geometry_msgs::msg::TransformStamped, double>(
			prior,
			resampleperiod,
			resamplethreshold,
			resamplescheme)
	{}

	vector<BFL::WeightedSample<geometry_msgs::msg::TransformStamped>> CustomParticleFilter::getNewSamples()
	{
		return _new_samples;
	}
} // nav2_localization
