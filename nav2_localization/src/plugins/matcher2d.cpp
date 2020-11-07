#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "nav2_localization/plugins/matcher2d_plugins.hpp"
#include "nav2_localization/map_utils.hpp"
#include "tf2/utils.h"

namespace nav2_localization
{

void LikelihoodFieldMatcher2dPDF::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
	node_ = node;

	node_->declare_parameter("max_likelihood_distance", 2.0);
	node_->declare_parameter("sigma_hit", 0.2);
	node_->declare_parameter("max_number_of_beams", 60);
	node_->declare_parameter("z_max", 0.05);
	node_->declare_parameter("z_min", 0.0);
	node_->declare_parameter("z_hit", 0.5);
	node_->declare_parameter("z_rand", 0.5);

	node_->get_parameter("max_likelihood_distance", max_likelihood_distace_);
    node_->get_parameter("sigma_hit", sigma_hit_);
	node_->get_parameter("max_number_of_beams", max_number_of_beams_);
	node_->get_parameter("z_max", z_max_);
	node_->get_parameter("z_min", z_min_);
	node_->get_parameter("z_hit", z_hit_);
	node_->get_parameter("z_rand", z_rand_);
}

BFL::Probability LikelihoodFieldMatcher2dPDF::ProbabilityGet(
	const sensor_msgs::msg::LaserScan &measurement) const
{
	double q = 1;

	// in case the user specfies more beams than is avaiable
	int max_number_of_beams = std::min(max_number_of_beams, static_cast<int>(laser_scan_->ranges.size()));
	for(int i=0; i<max_number_of_beams; i++)
	{
		if(laser_scan_->ranges[i]<z_max_ || laser_scan_->ranges[i]>z_min_) // within sensor range
		{
			geometry_msgs::msg::TransformStamped curr_pose = ConditionalArgumentGet(0);
			double x_z_kt = curr_pose.transform.translation.x +
							laser_pose_.transform.translation.x * cos(tf2::getYaw(curr_pose.transform.rotation)) -
							laser_pose_.transform.translation.y * sin(tf2::getYaw(curr_pose.transform.rotation)) +
							laser_scan_->ranges[i] * 
							cos(tf2::getYaw(curr_pose.transform.rotation) + tf2::getYaw(laser_pose_.transform.rotation));
			double y_z_kt = curr_pose.transform.translation.y +
							laser_pose_.transform.translation.y * cos(tf2::getYaw(curr_pose.transform.rotation)) +
							laser_pose_.transform.translation.x * sin(tf2::getYaw(curr_pose.transform.rotation)) +
							laser_scan_->ranges[i] * 
							sin(tf2::getYaw(curr_pose.transform.rotation) + tf2::getYaw(laser_pose_.transform.rotation));
			int8_t end_point_index = MapUtils::coordinates_to_index(x_z_kt, y_z_kt, map_->info.width);
			double dist_prob = pre_computed_likelihood_field_.at(end_point_index);

			q *= z_hit_* dist_prob + (z_rand_/z_max_);
		}
	}
	return BFL::Probability(q);
}

void LikelihoodFieldMatcher2dPDF::activate()
{

}

void LikelihoodFieldMatcher2dPDF::deactivate()
{

}

void LikelihoodFieldMatcher2dPDF::cleanup()
{

}

void LikelihoodFieldMatcher2dPDF::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
	map_ = map;
	preComputeLikelihoodField();	// TODO: is this the best place for this?
}

void LikelihoodFieldMatcher2dPDF::setLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan)
{
	laser_scan_ = laser_scan;
}

sensor_msgs::msg::LaserScan::ConstSharedPtr LikelihoodFieldMatcher2dPDF::getLaserScan()
{
	return laser_scan_;
}

void LikelihoodFieldMatcher2dPDF::preComputeLikelihoodField()
{
	std::vector<int> occupied_cells;

    RCLCPP_ERROR(node_->get_logger(), "1");

	// Identify all the occupied cells
	for(auto index=0; index < map_->info.width*map_->info.height; index++)
	{
		if(map_->data[index]==100) // the cell is occupied
		{
			pre_computed_likelihood_field_[index] = 0.0;
			occupied_cells.push_back(index);
		}
		else
			pre_computed_likelihood_field_[index] = max_likelihood_distace_;
	}

    RCLCPP_ERROR(node_->get_logger(), "2");

	// Depth first search for other cells
	for(auto index : occupied_cells)
	{
		std::vector<bool> visited(map_->info.width*map_->info.height, false);
		DFS(index, index, visited);
	}

    RCLCPP_ERROR(node_->get_logger(), "3");
	
	// Apply zero-mean norrmal distribution
	for(auto index=0; index < map_->info.width*map_->info.height; index++)
		pre_computed_likelihood_field_[index] = (1.0/(sqrt(2*M_PI)*sigma_hit_))*exp(-0.5*((pre_computed_likelihood_field_[index]*pre_computed_likelihood_field_[index])/(sigma_hit_*sigma_hit_)));
    RCLCPP_ERROR(node_->get_logger(), "4");
}

void LikelihoodFieldMatcher2dPDF::DFS(const int &index_curr, const int &index_of_obstacle, std::vector<bool> &visited)
{
	visited[index_curr] = true;
	std::pair<uint32_t, uint32_t> coord_curr = MapUtils::index_to_coordinates(index_curr, map_->info.width);
	std::pair<uint32_t, uint32_t> coord_obs = MapUtils::index_to_coordinates(index_of_obstacle, map_->info.width);

	// This cell is NOT an obstacle
	if(pre_computed_likelihood_field_[index_curr]!=0.0)	
	{
		double distance_to_obstacle = MapUtils::distance_between_two_points(coord_curr.first, coord_curr.second, coord_obs.first, coord_obs.second)*map_->info.resolution;

		// Getting far from the obstacle
		if(distance_to_obstacle > max_likelihood_distace_)
			return;

		// Found a closer obstacle
		if(distance_to_obstacle < pre_computed_likelihood_field_[index_curr])
			pre_computed_likelihood_field_[index_curr] = distance_to_obstacle;
	}

	// left
	if(coord_curr.first > 0)
	{
		int left_cell_index =  MapUtils::coordinates_to_index(coord_curr.first-1, coord_curr.second, map_->info.width);
		if(!visited[left_cell_index])
			DFS(left_cell_index, index_of_obstacle, visited);
	}

	// right
	if(coord_curr.first < map_->info.width-1)
	{
		int right_cell_index =  MapUtils::coordinates_to_index(coord_curr.first+1, coord_curr.second, map_->info.width);
		if(!visited[right_cell_index])
			DFS(right_cell_index, index_of_obstacle, visited);
	}

	// up
	if(coord_curr.second > 0)
	{
		int up_cell_index =  MapUtils::coordinates_to_index(coord_curr.first, coord_curr.second-1, map_->info.width);
		if(!visited[up_cell_index])
			DFS(up_cell_index, index_of_obstacle, visited);
	}

	// down
	if(coord_curr.second < map_->info.height-1)
	{
		int down_cell_index =  MapUtils::coordinates_to_index(coord_curr.first, coord_curr.second+1, map_->info.width);
		if(!visited[down_cell_index])
			DFS(down_cell_index, index_of_obstacle, visited);
	}
}

void LikelihoodFieldMatcher2dPDF::setLaserPose(const geometry_msgs::msg::TransformStamped& laser_pose)
{
	laser_pose_ = laser_pose;
}
} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::LikelihoodFieldMatcher2dPDF, nav2_localization::Matcher2dPDF)
