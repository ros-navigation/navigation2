#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solver_plugins.hpp"

namespace nav2_localization
{

geometry_msgs::msg::PoseWithCovariance DummySolver2d::solve(
	const nav_msgs::msg::Odometry& curr_odom,
	const sensor_msgs::msg::LaserScan& scan)
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
	motionSampler_ = motionSampler;
	matcher_ = matcher
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
