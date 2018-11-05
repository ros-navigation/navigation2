#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav2_costmap_2d/costmap_2d_ros.h>
#include <sstream>

using namespace std::chrono_literals;
nav2_dynamic_params::DynamicParamsClient * dynamic_params_client;

// Define a user event callback
void event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "Event Callback!");

  if (dynamic_params_client->is_in_event(event, "foo")) {
    RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "'foo' is in this event!");
  }

  double foo;
  dynamic_params_client->get_event_param_or("foo", foo, 2.5);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foo: %f", foo);

  int bar;
  dynamic_params_client->get_event_param_or("param_tests/bar", bar, 4);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "bar: %d", bar);

  std::vector<std::string> foobar;
  dynamic_params_client->get_event_param_or("foobar", foobar, {"default"}, "some_namespace");
  std::stringstream streams;
  for (auto & s : foobar) {
    streams << "\n" << s;
  }
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foobar: %s", streams.str().c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("my_other_node", "some_namespace");
  auto remote_nodes = node->get_node_names();
  
  std::string remote_node_name = "param_tests/my_node";

  // Add Dynamic Reconfigure Client
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node,
    {remote_node_name, "param_tests", "some_namespace/my_node", ""});

  // Add specific remote parameters by namespace
  //dynamic_params_client->add_parameters("", {"foo"});
  //dynamic_params_client->add_parameters("param_tests", {"bar"});
  //dynamic_params_client->add_parameters("some_namespace", {"foobar"});
  
  // adds all parameters on remote nodes
  dynamic_params_client->add_parameters();

  dynamic_params_client->set_callback(std::bind(event_callback, std::placeholders::_1));

  auto list = dynamic_params_client->get_param_names();

  std::stringstream ss;
  for (auto & param_name : list)
  {
    ss << "\n" << param_name;
  }

  RCLCPP_INFO(node->get_logger(), ss.str().c_str()); 
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
