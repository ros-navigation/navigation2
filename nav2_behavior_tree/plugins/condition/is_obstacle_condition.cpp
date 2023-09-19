#include <string>
#include <memory>
#include <utility>
#include "nav2_behavior_tree/plugins/condition/is_obstacle_condition.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree{

	using namespace std::chrono_literals;
	using namespace std::placeholders;

	IsObstacle::IsObstacle(const std::string & condition_name, const BT::NodeConfiguration & conf): BT::ConditionNode(condition_name, conf), distance_(1.0){
		node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node"); 
		callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
 		callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  		callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  		rclcpp::SubscriptionOptions sub_option;
  		sub_option.callback_group = callback_group_;
		
		laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>("scan", 100, std::bind(&IsObstacle::laser_callback, this, _1), sub_option);
		last_reading_time_ = node_->now();
	}


	void IsObstacle::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg){
		last_scan_ = std::move(msg);
	}

	BT::NodeStatus IsObstacle::tick(){
		if (last_scan_ == nullptr) {
			return BT::NodeStatus::FAILURE;
		}
		
		getInput("distance", distance_);
		
		for(int i = 0; i < int(last_scan_->ranges.size()); i++){
			if (last_scan_->ranges[i] < distance_) {
				return BT::NodeStatus::SUCCESS;
			} 
		}
		return BT::NodeStatus::FAILURE;
		
	}
}


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory){
	factory.registerNodeType<nav2_behavior_tree::IsObstacle>("IsObstacle");
}
