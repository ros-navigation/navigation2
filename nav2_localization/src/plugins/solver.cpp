#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"
#include "nav2_localization/custom_particle_filter.hpp"
#include <vector>

namespace nav2_localization
{
void DummySolver2d::CreateParticleFilter(
	unsigned int NUM_SAMPLES,
	unsigned int STATE_SIZE,
	float PRIOR_MU_X,
	float PRIOR_MU_Y,
	float PRIOR_MU_THETA,
	float PRIOR_COV_X,
	float PRIOR_COV_Y,
	float PRIOR_COV_THETA)
{
	// Convert orientation from Euler Angles to Quaternion
	float roll = 0.0, pitch = 0.0, yaw = PRIOR_MU_THETA;
	float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    float q_x = cr * cp * cy + sr * sp * sy;
    float q_y = sr * cp * cy - cr * sp * sy;
    float q_z = cr * sp * cy + sr * cp * sy;
    float q_w = cr * cp * sy - sr * sp * cy;

	// Generate a particle for initialization
	geometry_msgs::msg::TransformStamped default_pose;
	default_pose.transform.translation.x = PRIOR_MU_X;
	default_pose.transform.translation.y = PRIOR_MU_Y;
	default_pose.transform.translation.z = 0.0;
	default_pose.transform.rotation.x = q_x;
	default_pose.transform.rotation.y = q_y;
	default_pose.transform.rotation.z = q_z;
	default_pose.transform.rotation.w = q_w;

	// Create vector of particles
	// TODO - Add noise to each particle using covariance
	prior_samples_ = std::make_shared<std::vector<geometry_msgs::msg::TransformStamped>>;
	for(int i=0; i<NUM_SAMPLES; i++)
		prior_samples_->push_back(default_pose);

	// Create particle filter
	pf_ = new CustomParticleFilter(prior=prior_samples_, resampleperiod=0, resample_threshold=NUM_SAMPLES/4.0);
}

geometry_msgs::msg::TransformStamped DummySolver2d::solve(
	const geometry_msgs::msg::TransformStamped& curr_odom)
{
	// Motion update with motion sampler and current odometry
	pf_->updateParticles(motionSampler_, curr_odom);

	// Weigths calculation with matcher and measurement
	pf_->computeWeights(matcher_, *(matcher_->getLaserScan()));
    

	// Resample - Future TODO - How to choose resampling method? One per function, argument...?
	pf_->resample();

	// Get new particles (in case we want to publish them)
	// samples = pf_->getParticles();

	// Returns an estimated pose using all the information contained in the particle filter.
	// TODO - Add covariance to TransformStamped msg
    geometry_msgs::msg::TransformStamped* pose = pf_->getPosterior();

    return pose;
}

void DummySolver2d::configure(
	const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
	SampleMotionModelPDF::Ptr& motionSampler,
	Matcher2dPDF::Ptr& matcher,
	const geometry_msgs::msg::TransformStamped& odom,
	const geometry_msgs::msg::Pose& pose)
{
	node_ = node;

	node_->declare_parameter("num_particles", 1000);
	node_->declare_parameter("num_dimensions", 3);
	node_->declare_parameter("prior_mu_x", 0.0);
	node_->declare_parameter("prior_mu_y", 0.0);
	node_->declare_parameter("prior_mu_theta", 0.0);
	node_->declare_parameter("prior_cov_x", 0.0);
	node_->declare_parameter("prior_cov_y", 0.0);
	node_->declare_parameter("prior_cov_theta", 0.0);

	motionSampler_ = motionSampler;
	matcher_ = matcher;

	// Get configuration and generate PF
	int NUM_SAMPLES;
	int STATE_SIZE;
	double PRIOR_MU_X;
	double PRIOR_MU_Y;
	double PRIOR_MU_THETA;
	double PRIOR_COV_X;
	double PRIOR_COV_Y;
	double PRIOR_COV_THETA;
	node_->get_parameter("num_particles", NUM_SAMPLES);
	node_->get_parameter("num_dimensions", STATE_SIZE); // TODO - Can we fix it to 3? (or other, depending on plugin)
	node_->get_parameter("prior_mu_x", PRIOR_MU_X);
	node_->get_parameter("prior_mu_y", PRIOR_MU_Y);
	node_->get_parameter("prior_mu_theta", PRIOR_MU_THETA);
	node_->get_parameter("prior_cov_x", PRIOR_COV_X);
	node_->get_parameter("prior_cov_y", PRIOR_COV_Y);
	node_->get_parameter("prior_cov_theta", PRIOR_COV_THETA);
	CreateParticleFilter(NUM_SAMPLES, STATE_SIZE, PRIOR_MU_X, PRIOR_MU_Y, PRIOR_MU_THETA, PRIOR_COV_X, PRIOR_COV_Y, PRIOR_COV_THETA);

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
