#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav2_costmap_2d/costmap_2d_ros.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("param_tests");

  node->set_parameters({rclcpp::Parameter("foo", 1.0), rclcpp::Parameter("bar", 2)});
  std::vector<std::string> list = {"hello", "goodbye"};
  node->set_parameters({rclcpp::Parameter("foobar", list)});
  
  // Can't set a list of lists as a parameter in ROS2
  std::vector<std::vector<int>> list_of_lists = {{1, 2}, {3, 4}};
  //node->set_parameters({rclcpp::Parameter("baz", list_of_lists)});

  auto node_2 = rclcpp::Node::make_shared("my_node", node->get_name());
  node_2->set_parameters({rclcpp::Parameter("foobar", list)});

  auto node_3 = rclcpp::Node::make_shared("my_other_node", "some_namespace");
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node_3, node_2->get_name());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(node_2);
  exec.add_node(node_3);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
