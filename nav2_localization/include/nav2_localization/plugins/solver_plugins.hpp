#ifndef NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_

// Interfaces
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/interfaces/motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/poseWithCovariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/LaserScan.hpp"
#include "nav_msgs/msg/OccupancyGrid.hpp"

namespace nav2_localization_plugins
{
class DummySolver2d : public nav2_localization_base::Solver
{
public:
	DummySolver2d(){}

	geometry_msgs::msg::PoseWithCovariance localize(
		const nav2_localization_base::SampleMotionModel& motionSampler,
		const nav2_localization_base::Matcher2d& matcher,
		const nav_msgs::msg::Odometry& prev_odom,
		const nav_msgs::msg::Odometry& curr_odom,
		const geometry_msgs::msg::Pose& prev_pose,
		const sensor_msgs::msg::LaserScan& scan,
		const nav_msgs::msg::OccupancyGrid& map);
};
} // nav2_localization_plugins

#endif // NAV2_LOCALIZATION__SOLVER_PLUGINS_HPP_
