#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"
#include "nav2_localization/custom_particle_filter.hpp"
#include <model/systemmodel.h>
#include <model/measurementmodel.h>

namespace nav2_localization
{
void CreateSystemModel()
{
	/****************************************
	 * NonLinear system model (MotionModel) *
	 ****************************************/
	
	// create gaussian
	MatrixWrapper::ColumnVector sys_noise_Mu(BFL::STATE_SIZE);
	sys_noise_Mu(1) = BFL::MU_SYSTEM_NOISE_X;
	sys_noise_Mu(2) = BFL::MU_SYSTEM_NOISE_Y;
	sys_noise_Mu(3) = BFL::MU_SYSTEM_NOISE_THETA;
	
	MatrixWrapper::SymmetricMatrix sys_noise_Cov(BFL::STATE_SIZE);
	sys_noise_Cov = 0.0;
	sys_noise_Cov(1,1) = BFL::SIGMA_SYSTEM_NOISE_X;
	sys_noise_Cov(1,2) = 0.0;
	sys_noise_Cov(1,3) = 0.0;
	sys_noise_Cov(2,1) = 0.0;
	sys_noise_Cov(2,2) = BFL::SIGMA_SYSTEM_NOISE_Y;
	sys_noise_Cov(2,3) = 0.0;
	sys_noise_Cov(3,1) = 0.0;
	sys_noise_Cov(3,2) = 0.0;
	sys_noise_Cov(3,3) = BFL::SIGMA_SYSTEM_NOISE_THETA;
	
	BFL::Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);
	
	// create the nonlinear system model
	sys_pdf = new NonlinearSystemPdf(system_Uncertainty); //TODO - Create the motion model instead (check args)
	sys_model = new SystemModel<MatrixWrapper::ColumnVector> (sys_pdf);

	return sys_model;
}	
	
void CreateMeasurementModel()
{
	/*****************************************
	 * NonLinear Measurement model (Matcher) *
	 *****************************************/
	
	// Construct the measurement noise (a scalar in this case)
	MatrixWrapper::ColumnVector meas_noise_Mu(BFL::MEAS_SIZE);
	meas_noise_Mu(1) = MU_MEAS_NOISE;
	meas_noise_Mu(2) = MU_MEAS_NOISE;
	meas_noise_Mu(3) = MU_MEAS_NOISE;
	MatrixWrapper::SymmetricMatrix meas_noise_Cov(BFL::MEAS_SIZE);
	meas_noise_Cov(1,1) = BFL::SIGMA_MEAS_NOISE;
	meas_noise_Cov(1,2) = 0.0;
	meas_noise_Cov(1,3) = 0.0;
	meas_noise_Cov(2,1) = 0.0;
	meas_noise_Cov(2,2) = BFL::SIGMA_MEAS_NOISE;
	meas_noise_Cov(2,3) = 0.0;
	meas_noise_Cov(3,1) = 0.0;
	meas_noise_Cov(3,2) = 0.0;
	meas_noise_Cov(3,3) = BFL::SIGMA_MEAS_NOISE;
	
	BFL::Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);
	
	// create the nonlinear measurement model
	meas_pdf = new NonlinearMeasurementPdf(measurement_Uncertainty, map_); //TODO - Create the matcher instead (check args)
	meas_model = new MeasurementModel<MatrixWrapper::ColumnVector,MatrixWrapper::ColumnVector>(meas_pdf);

	return meas_model;
	
void CreateParticleFilter()
{
	/****************************
	 * Linear prior DENSITY     *
	 ***************************/
	// Continuous Gaussian prior (for Kalman filters)
	MatrixWrapper::ColumnVector prior_Mu(BFL::STATE_SIZE);
	prior_Mu(1) = BFL::PRIOR_MU_X;
	prior_Mu(2) = BFL::PRIOR_MU_Y;
	prior_Mu(3) = BFL::PRIOR_MU_THETA;
	MatrixWrapper::SymmetricMatrix prior_Cov(BFL::STATE_SIZE);
	prior_Cov(1,1) = BFL::PRIOR_COV_X;
	prior_Cov(1,2) = 0.0;
	prior_Cov(1,3) = 0.0;
	prior_Cov(2,1) = 0.0;
	prior_Cov(2,2) = BFL::PRIOR_COV_Y;
	prior_Cov(2,3) = 0.0;
	prior_Cov(3,1) = 0.0;
	prior_Cov(3,2) = 0.0;
	prior_Cov(3,3) = BFL::PRIOR_COV_THETA;
	BFL::Gaussian prior_cont(prior_Mu, prior_Cov);
	
	// Discrete prior for Particle filter (using the continuous Gaussian prior)
	vector<Sample<MatrixWrapper::ColumnVector> > prior_samples(BFL::NUM_SAMPLES);
	prior_discr = new BFL::MCPdf<MatrixWrapper::ColumnVector>(BFL::NUM_SAMPLES, BFL::STATE_SIZE);
	prior_cont.SampleFrom(prior_samples, BFL::NUM_SAMPLES, BFL::CHOLESKY, NULL);
	prior_discr->ListOfSamplesSet(prior_samples);
	
	/******************************
	 * Construction of the Filter *
	 ******************************/
	//filter = new CustomParticleFilter(prior_discr, 0.5, BFLNUM_SAMPLES/4.0);
	filter = CustomParticleFilter(prior_discr, 0.5, BFLNUM_SAMPLES/4.0);

	return filter;
}

DummySolver2d::DummySolver2d() {}

geometry_msgs::msg::PoseWithCovariance DummySolver2d::solve(
	const nav_msgs::msg::Odometry& curr_odom)
{
	// STEP 1 - Motion update
	for(unsigned int i=0; i<pf.n_particles; i++)
	{
		pf.particles[i].pose = motionSampler_.getMostLikelyPose(prev_odom, curr_odom, pf.particles[i].pose); // TODO - Substitute 2 odoms for the transformation
		// NOTE: The motion sampler (for MCL-based localizations) will get each particle's pose. No need to store "previous pose"?
	}

	// STEP 2 - Weight calculation
	for(unsigned int i=0; i<pf.n_particles; i++)
	{
		pf.particles[i].weight = matcher_.match(scan, pf.particles[i].pose);
		// NOTE: Send only scan and use scan+map to return most likely pose and compute the weight here??
		// NOTE: As Steve said, to delegate as much as possible, the weight/score/likelihood could be computed in "match()" and, if needed, refined here
		// NOTE: Will the weight/score/likelihood work for graph or KF based approaches?
	}

	// STEP 3 - Resample (using the already updated weights)
	pf.resample();

	// STEP 4 - Return a most likely pose
	// TODO - Decide how
	//   - Pose of particle with best score -> Chosen for now for simplicity
	//   - Weighted mean of X best scores
	//   - ???

	float max_weight = 0;
	int best_particle_idx = -1;	
	for(unsigned int i=0; i<pf.n_particles; i++)
	{
		if(pf.particles[i].weight > max_weight)
		{
			max_weight = pf.particles[i].weight;
			best_particle_idx = i;
		}
	}

    return pf.particles[best_particle_idx];
}

void DummySolver2d::configure(
	const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
	SampleMotionModel& motionSampler,
	Matcher2d& matcher,
	const nav_msgs::msg::Odometry& odom,
	const geometry_msgs::msg::Pose& pose)
{
	// TODO - Generate particle filter, sample the map randomly or use initial pose?
	node_ = node;
	pf_ = CreateParticleFilter();
	motionSampler_ = motionSampler;
	matcher_ = matcher;
	prev_odom_ = odom;
	prev_pose_ = pose;
	return;
}

void DummySolver2d::activate()
{

}

void DummySolver2d::deactivate()
{

}

void DummySolver2d::cleanup()
{

}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummySolver2d, nav2_localization::Solver)
