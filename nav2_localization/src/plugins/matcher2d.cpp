#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include "nav2_localization/plugins/matcher2d_plugins.hpp"
#include "nav2_localization/map_utils.hpp"
#include "tf2/utils.h"

namespace nav2_localization
{

void LikelihoodFieldMatcher2d::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
	node_ = node;

	node_->declare_parameter("max_likelihood_distance", 2.0);
	node_->declare_parameter("sigma_hit", 0.2);
	node_->declare_parameter("max_number_of_beams", 60);
	node_->declare_parameter("z_hit", 0.5);
	node_->declare_parameter("z_rand", 0.5);

	node_->get_parameter("max_likelihood_distance", max_likelihood_distace_);
    node_->get_parameter("sigma_hit", sigma_hit_);
	node_->get_parameter("max_number_of_beams", max_number_of_beams_);
	node_->get_parameter("z_hit", z_hit_);
	node_->get_parameter("z_rand", z_rand_);
}

double LikelihoodFieldMatcher2d::getScanProbability(
	const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
	const geometry_msgs::msg::TransformStamped &curr_pose)
{
	double q = 1;	// Probability of curr_pose given scan
	double theta = tf2::getYaw(curr_pose.transform.rotation); // Robot's orientation

	double z_max = scan->range_max;
	double z_min = scan->range_min;

	// In case the user specifies more beams than is available
	int max_number_of_beams = std::min(max_number_of_beams, static_cast<int>(scan->ranges.size()));
	int beams_to_skip = scan->ranges.size()/max_number_of_beams;
	
	// Iterate over the specfied number of beams, skipping between them to get an even distribution  
	for(int i=0; i<max_number_of_beams; i+=beams_to_skip)
	{
		// Check if the range is within the sensor's limits
		if(scan->ranges[i]<z_max && scan->ranges[i]>z_min)
		{
			// Map the beam end-point onto the map
			double beam_angle = tf2::getYaw(sensor_pose_.transform.rotation) + scan->angle_min + scan->angle_increment*i;
			double x_z_kt = curr_pose.transform.translation.x +
							sensor_pose_.transform.translation.x * cos(theta) -
							sensor_pose_.transform.translation.y * sin(theta) +
							scan->ranges[i] * 
							cos(theta + beam_angle);
			double y_z_kt = curr_pose.transform.translation.y +
							sensor_pose_.transform.translation.y * cos(theta) +
							sensor_pose_.transform.translation.x * sin(theta) +
							scan->ranges[i] * 
							sin(theta + beam_angle);

			// Get index of the laser end-point in the grid map
			int end_point_index = MapUtils::coordinates_to_index(x_z_kt, y_z_kt, map_->info.width);

			// Get the likelihood field probability at that endpoint 
			double dist_prob = pre_computed_likelihood_field_[end_point_index];

			q *= z_hit_* dist_prob + (z_rand_/z_max);
		}
	}
	return q;
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

void LikelihoodFieldMatcher2d::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
	map_ = map;
	preComputeLikelihoodField();	// TODO: is this the best place for this?
}

void LikelihoodFieldMatcher2d::preComputeLikelihoodField()
{
	std::vector<int> occupied_cells;

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

    // Depth first search for other cells
	for(auto index : occupied_cells)
	{
		std::vector<bool> visited(map_->info.width*map_->info.height, false);
		DFS(index, index, visited);
	}

    // Apply zero-mean norrmal distribution
	for(auto index=0; index < map_->info.width*map_->info.height; index++)
		pre_computed_likelihood_field_[index] = (1.0/(sqrt(2*M_PI)*sigma_hit_))*exp(-0.5*((pre_computed_likelihood_field_[index]*pre_computed_likelihood_field_[index])/(sigma_hit_*sigma_hit_)));
}

void LikelihoodFieldMatcher2d::DFS(const int &index_curr, const int &index_of_obstacle, std::vector<bool> &visited)
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

void LikelihoodFieldMatcher2d::setSensorPose(const geometry_msgs::msg::TransformStamped &sensor_pose)
{
	sensor_pose_ = sensor_pose;
}
} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::LikelihoodFieldMatcher2d, nav2_localization::Matcher2d)
