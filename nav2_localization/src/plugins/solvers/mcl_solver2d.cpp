#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/plugins/solvers/mcl_solver2d.hpp"
#include "nav2_localization/particle_filter.hpp"
#include "nav2_localization/angle_utils.hpp"
#include "tf2/utils.h"
#include <vector>

namespace nav2_localization
{
geometry_msgs::msg::TransformStamped MCLSolver2d::solve(
	const geometry_msgs::msg::TransformStamped& curr_odom,
	const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan)
{
	if(first_iteration_)
	{
		prev_odom_ = curr_odom;
		first_iteration_ = false;
	}
	double distance_moved = std::hypot(prev_odom_.transform.translation.x-curr_odom.transform.translation.x,
									   prev_odom_.transform.translation.y-curr_odom.transform.translation.y);
	double angle_moved = AngleUtils::angleDiff(tf2::getYaw(curr_odom.transform.rotation), tf2::getYaw(prev_odom_.transform.rotation));
	if((distance_moved < motion_linear_tol_) && (angle_moved < motion_angular_tol_))
		return prev_pose_;

	pf_->update(prev_odom_, curr_odom, scan, motionSampler_, matcher_);
	prev_odom_ = curr_odom;

	geometry_msgs::msg::TransformStamped curr_pose = pf_-> getMostLikelyPose();
	prev_pose_ = curr_pose;

    return curr_pose;
}

void MCLSolver2d::initFilter(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &pose)
{
	geometry_msgs::msg::TransformStamped init_pose;
	init_pose.transform.translation.x = pose->pose.pose.position.x;
	init_pose.transform.translation.y = pose->pose.pose.position.y;
	init_pose.transform.translation.z = pose->pose.pose.position.z;
	init_pose.transform.rotation = pose->pose.pose.orientation;
	pf_->initFilter(init_number_of_particles_, init_pose);

	first_iteration_ = true;
	prev_pose_ = init_pose;
}

void MCLSolver2d::configure(
	const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
	SampleMotionModel::Ptr& motionSampler,
	Matcher2d::Ptr& matcher,
	const geometry_msgs::msg::TransformStamped& odom,
	const geometry_msgs::msg::TransformStamped& pose)
{
	node_ = node;

	node_->declare_parameter("num_particles", 1000);
	node_->declare_parameter("motion_linear_tol", 0.05);
	node_->declare_parameter("motion_angular_tol", 0.05);

	motionSampler_ = motionSampler;
	matcher_ = matcher;

	// Get configuration and generate PF
	node_->get_parameter("num_particles", init_number_of_particles_);
	node_->get_parameter("motion_linear_tol", motion_linear_tol_);
	node->get_parameter("motion_angular_tol", motion_angular_tol_);

	pf_ = std::make_shared<ParticleFilter>(init_number_of_particles_, pose);

	prev_pose_ = pose;
	return;
}

void MCLSolver2d::activate()
{

}

void MCLSolver2d::deactivate()
{

}

void MCLSolver2d::cleanup()
{

}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::MCLSolver2d, nav2_localization::Solver)
