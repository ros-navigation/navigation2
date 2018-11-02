#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav2_costmap_2d/costmap_2d_ros.h>
#include <sstream>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_1 = rclcpp::Node::make_shared("param_tests");
  auto node_2 = rclcpp::Node::make_shared("my_node", node_1->get_name());
  auto node_3 = rclcpp::Node::make_shared("my_node", "some_namespace");

  node_1->set_parameters({rclcpp::Parameter("foo", 1.0), rclcpp::Parameter("bar", 2)});
  std::vector<std::string> list = {"hello", "goodbye"};
  node_1->set_parameters({rclcpp::Parameter("foobar", list)});
  
  // Can't set a list of lists as a parameter in ROS2
  //std::vector<std::vector<int>> list_of_lists = {{1, 2}, {3, 4}};
  //node->set_parameters({rclcpp::Parameter("baz", list_of_lists)});

  node_2->set_parameters({rclcpp::Parameter("foobar", list)});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_1);
  exec.add_node(node_2);
  exec.add_node(node_3);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
