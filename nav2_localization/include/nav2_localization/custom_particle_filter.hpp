#ifndef NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_

#include <filter/bootstrapfilter.h>

namespace nav2_localization
{
class CustomParticleFilter : public BFL::BootstrapFilter<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
	CustomParticleFilter(
		BFL::MCPdf<MatrixWrapper::ColumnVector> * prior,
		int resampleperiod = 0,
		double resamplethreshold = 0,
		int resamplescheme = DEFAULT_RS);

	vector<WeightedSample<MatrixWrapper::ColumnVector> > getNewSamples();

};
} // nav2_localization

#endif // NAV2_LOCALIZATION__CUSTOM_PARTICLE_FILTER_HPP_
