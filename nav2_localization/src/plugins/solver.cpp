#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"
#include "nav2_localization/custom_particle_filter.hpp"
#include <model/systemmodel.h>
#include <model/measurementmodel.h>

namespace nav2_localization
{
void CreateParticleFilter(unsigned int NUM_SAMPLES, unsigned int STATE_SIZE, float PRIOR_MU_X, float PRIOR_MU_Y, float PRIOR_MU_THETA, float PRIOR_COV_X, float PRIOR_COV_Y, float PRIOR_COV_THETA)
{
	/****************************
	 * Linear prior DENSITY     *
	 ***************************/
	// Continuous Gaussian prior (for Kalman filters)
	//// ColumnVector and SymmetricMatrix are used because they are what BFL::Gaussian requires
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
	//prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
	prior_cont.SampleFrom(prior_samples, NUM_SAMPLES); // Use default values for "method" and "args" arguments
	prior_discr->ListOfSamplesSet(prior_samples);
	
	/******************************
	 * Construction of the Filter *
	 ******************************/
	filter = new CustomParticleFilter(prior_discr, 0.5, NUM_SAMPLES/4.0);

	return filter;
}

DummySolver2d::DummySolver2d() {}

geometry_msgs::msg::TransformStamped DummySolver2d::solve(
	const geometry_msgs::msg::TransformStamped& curr_odom)
{
	// Motion update with motion sampler and current odometry
	pf_->Update(motionSampler_, curr_odom);

	// Weigths calculation with matcher and measurement
	pf_->Update(matcher_, matcher_->getLaserScan());

	// Get new particles (in case we want to publish them)
	samples = pf_->getNewSamples();

	// Returns an estimated pose using all the information contained in the particle filter.
	// TODO - Add covariance to TransformStamped msg
    BFL::Pdf<geometry_msgs::msg::TransformStamped>* posterior = pf_->PostGet();
    geometry_msgs::msg::TransformStamped pose = posterior->ExpectedValueGet();
    SymmetricMatrix pose_cov = posterior->CovarianceGet();

    return pose;
}

void DummySolver2d::configure(
	const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
	SampleMotionModelPDF::Ptr& motionSamplerPDF,
	Matcher2dPDF::Ptr& matcherPDF,
	const nav_msgs::msg::Odometry& odom,
	const geometry_msgs::msg::Pose& pose)
{
	node_ = node;

	// Get configuration and generate PF
	unsigned int NUM_SAMPLES;
	unsigned int STATE_SIZE;
	float PRIOR_MU_X;
	float PRIOR_MU_Y;
	float PRIOR_MU_THETA;
	float PRIOR_COV_X;
	float PRIOR_COV_Y;
	float PRIOR_COV_THETA;
	node_->get_parameter("num_particles", NUM_SAMPLES);
	node_->get_parameter("num_dimensions", STATE_SIZE); // TODO - Can we fix it to 3? (or other, depending on plugin)
	node_->get_parameter("prior_mu_x", PRIOR_MU_X);
	node_->get_parameter("prior_mu_y", PRIOR_MU_Y);
	node_->get_parameter("prior_mu_theta", PRIOR_MU_THETA);
	node_->get_parameter("prior_cov_x", PRIOR_COV_X);
	node_->get_parameter("prior_cov_y", PRIOR_COV_Y);
	node_->get_parameter("prior_cov_theta", PRIOR_COV_THETA);
	pf_ = CreateParticleFilter(NUM_SAMPLES, STATE_SIZE, PRIOR_MU_X, PRIOR_MU_Y, PRIOR_MU_THETA, PRIOR_COV_X, PRIOR_COV_Y, PRIOR_COV_THETA);

	motionSampler_.SystemPdfSet(motionSamplerPDF.get());
	matcher_.MeasurementPdfSet(matcherPDF.get());
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
