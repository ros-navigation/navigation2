#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "nav2_localization/plugins/matcher2d_plugins.hpp"
#include "pdfs/likelihood_field_pdf.hpp"

namespace nav2_localization
{

void LikelihoodFieldMatcher2d::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
	node_ = node;
	node_->get_parameter("max_likelihood_distance", max_likelihood_distace_);
	node_->get_parameter("sigma_hit", sigma_hit_);
	node_->get_parameter("max_number_of_beams", max_number_of_beams_);
	node_->get_parameter("z_max", z_max_);
	node_->get_parameter("z_min", z_min_);
	node_->get_parameter("z_min", z_hit_);
	node_->get_parameter("z_min", z_rand_);

	// TODO: what if the laser pose/map are not set yet?
	auto measurement_pdf = std::make_unique<LikelihoodFieldPdf>(max_number_of_beams_,
																max_likelihood_distace_,
																sigma_hit_,
																z_max_,
																z_min_,
																z_rand_,
																node_,
																map_,
																laser_pose_);
	this->MeasurementPdfSet(measurement_pdf.get());
}

void LikelihoodFieldMatcher2d::activate()
{

}

void LikelihoodFieldMatcher2d::deactivate()
{

}

void LikelihoodFieldMatcher2d::cleanup()
{

}

void LikelihoodFieldMatcher2d::setMap(nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
	map_ = map;
}

void LikelihoodFieldMatcher2d::setLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan)
{
	laser_scan_ = laser_scan;
}

sensor_msgs::msg::LaserScan::ConstSharedPtr LikelihoodFieldMatcher2d::getLaserScan()
{
	return laser_scan_;
}

void LikelihoodFieldMatcher2d::setLaserPose(const geometry_msgs::msg::TransformStamped& laser_pose)
{
	laser_pose_ = laser_pose;
}
} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::LikelihoodFieldMatcher2d, nav2_localization::Matcher2d)
