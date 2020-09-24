#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"
#include "nav2_localization/custom_particle_filter.hpp"
#include <model/systemmodel.h>
#include <model/measurementmodel.h>

// TODO - Get config parameters
// STATE_SIZE
// PRIOR_MU_X
// PRIOR_MU_Y
// PRIOR_MU_THETA
// PRIOR_COV_X
// PRIOR_COV_Y
// PRIOR_COV_THETA
// NUM_SAMPLES
// CHOLESKY

namespace nav2_localization
{
void CreateParticleFilter()
{
	/****************************
	 * Linear prior DENSITY     *
	 ***************************/
	// Continuous Gaussian prior (for Kalman filters)
	// TODO - Use TransformStamped as prior and get the initialization
	MatrixWrapper::ColumnVector prior_Mu(STATE_SIZE);
	prior_Mu(1) = PRIOR_MU_X;
	prior_Mu(2) = PRIOR_MU_Y;
	prior_Mu(3) = PRIOR_MU_THETA;
	MatrixWrapper::SymmetricMatrix prior_Cov(STATE_SIZE);
	prior_Cov(1,1) = PRIOR_COV_X;
	prior_Cov(1,2) = 0.0;
	prior_Cov(1,3) = 0.0;
	prior_Cov(2,1) = 0.0;
	prior_Cov(2,2) = PRIOR_COV_Y;
	prior_Cov(2,3) = 0.0;
	prior_Cov(3,1) = 0.0;
	prior_Cov(3,2) = 0.0;
	prior_Cov(3,3) = PRIOR_COV_THETA;
	BFL::Gaussian prior_cont(prior_Mu, prior_Cov);
	
	// Discrete prior for Particle filter (using the continuous Gaussian prior)
	vector<BFL::Sample<geometry_msgs::msg::TransformStamped>> prior_samples(NUM_SAMPLES);
	prior_discr = new BFL::MCPdf<geometry_msgs::msg::TransformStamped>(NUM_SAMPLES, STATE_SIZE);
	prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
	prior_discr->ListOfSamplesSet(prior_samples);
	
	/******************************
	 * Construction of the Filter *
	 ******************************/
	filter = new CustomParticleFilter(prior_discr, 0.5, NUM_SAMPLES/4.0);

	return filter;
}

DummySolver2d::DummySolver2d() {}

geometry_msgs::msg::TransformStamped DummySolver2d::solve(
	const nav_msgs::msg::Odometry& curr_odom)
{
	/*
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
	*/

	// TODO - What is the input? Change in X and Y? Build a transformStamped from curr_odom?
	pf_->Update(motionSampler_, input);

	// TODO - What is the measurement? We use double, shouldnt it be a scan msg?
	// Should the solver get the measurement as input then?
	pf_->Update(matcher_, measurement);

	// Get new particles (in case we want to publish them)
	samples = pf_->getNewSamples();

	// Returns an estimated pose using all the information contained in the particle filter.
    BFL::Pdf<geometry_msgs::msg::TransformStamped>* posterior = pf_->PostGet();
    geometry_msgs::msg::TransformStamped pose = posterior->ExpectedValueGet();
    SymmetricMatrix pose_cov = posterior->CovarianceGet();

    return pose;
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
