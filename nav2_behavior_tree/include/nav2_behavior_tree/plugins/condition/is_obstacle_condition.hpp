#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS__OBSTACLE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS__OBSTACLE_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree{

	class IsObstacle : public BT::ConditionNode {
		public:
			IsObstacle(const std::string & condition_name, const BT::NodeConfiguration & conf);
			
			IsObstacle() = delete;
			
			BT::NodeStatus tick() override;
			
			static BT::PortsList providedPorts(){
				return BT::PortsList({BT::InputPort<double>("distance")});
			}
			
			void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
			
		private:
			double distance_;
			rclcpp::CallbackGroup::SharedPtr callback_group_;
  			rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  			std::thread callback_group_executor_thread;
			rclcpp::Node::SharedPtr node_;
			rclcpp::Time last_reading_time_;
			rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
			sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
	};
}

#endif	// NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OBSTACLE_CONDITION_HPP_
