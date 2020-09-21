#ifndef NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_

#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
class LikelihoodFieldMatcher2d : public nav2_localization::Matcher2d
{
public:
	LikelihoodFieldMatcher2d(){}

	void activate();
	void deactivate();
	void cleanup();

	void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

	void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr& map);

	void setLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan);

	sensor_msgs::msg::LaserScan::ConstSharedPtr getLaserScan();

	void setLaserPose(const geometry_msgs::msg::TransformStamped& laser_pose);

private:
	int max_number_of_beams_;
	double max_likelihood_distace_; // the distance beyond which the likelihood is 0
	double sigma_hit_;
	double z_max_;
	double z_min_;
	double z_hit_;
	double z_rand_;
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
