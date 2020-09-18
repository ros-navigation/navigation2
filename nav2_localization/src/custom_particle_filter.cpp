#include "nav2_localization/custom_particle_filter.hpp"

namespace  nav1_localization
{
	CustomParticleFilter::CustomParticleFilter(
		BFL::MCPdf<MatrixWrapper::ColumnVector> *prior,
		int resampleperiod,
		double resamplethreshold,
		int resamplescheme):
		BFL::BootstrapFilter<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>(
			prior,
			resampleperiod,
			resamplethreshold,
			resamplescheme)
	{}

	vector<BFL::WeightedSample<MatrixWrapper::ColumnVector>> CustomParticleFilter::getNewSamples()
	{
		return _new_samples;
	}
} // nav2_localization
