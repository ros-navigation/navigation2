#ifndef NAV2_LOCALIZATION__LIKELIHOOD_FIELD_PDF_HPP_
#define NAV2_LOCALIZATION__LIKELIHOOD_FIELD_PDF_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include <pdf/conditionalpdf.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <unordered_map>

namespace nav2_localization
{
class LikelihoodFieldPdf : public BFL::ConditionalPdf
    <double, geometry_msgs::msg::TransformStamped>
{
public:
    LikelihoodFieldPdf(const int &max_number_of_beams,
                       const double &max_likelihood_distace,
                       const double &sigma_hit,
                       const double &z_max,
                       const double &z_min,
                       const double &z_rand,
                       const nav2_util::LifecycleNode::SharedPtr &node,
                       const nav_msgs::msg::OccupancyGrid::SharedPtr &map,
                       const geometry_msgs::msg::TransformStamped &laser_pose);

    BFL::Probability ProbabilityGet(const sensor_msgs::msg::LaserScan &measurement) const;

private:
    std::unordered_map<int8_t, double>	pre_computed_likelihood_field_;
	int max_number_of_beams_;
	double max_likelihood_distace_; // the distance beyond which the likelihood is 0
	double sigma_hit_;
	double z_max_;
	double z_min_;
	double z_hit_;
	double z_rand_;
    nav2_util::LifecycleNode::SharedPtr node_;
	nav_msgs::msg::OccupancyGrid::SharedPtr map_; // Reference to the 2D occupancy grid map of the environment where the robot is
	geometry_msgs::msg::TransformStamped laser_pose_;

    void preComputeLikelihoodField();
	void DFS(const int &index_curr, const int &index_of_obstacle, std::vector<bool> &visited);
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__LIKELIHOOD_FIELD_PDF_HPP_