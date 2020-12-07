#ifndef NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_PLUGINS_HPP_

#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <unordered_map>

namespace nav2_localization
{
class LikelihoodFieldMatcher2d : public Matcher2d
{
public:
	LikelihoodFieldMatcher2d() : Matcher2d(){}

	void activate();
	void deactivate();
	void cleanup();

	double probabilityGet(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan, const sensor_msgs::msg::LaserScan & measurement, const geometry_msgs::msg::TransformStamped & curr_pose) const;

	void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

	void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);

	void setLaserPose(const geometry_msgs::msg::TransformStamped& laser_pose);

private:
	void preComputeLikelihoodField();	
	void DFS(const int &index_curr, const int &index_of_obstacle, std::vector<bool> &visited); //Deep First Search

	std::unordered_map<int, double>	pre_computed_likelihood_field_;
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
