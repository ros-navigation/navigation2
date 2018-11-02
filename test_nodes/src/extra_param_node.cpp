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
  dynamic_params_client->get_event_param(event, "foo", foo);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foo: %f", foo);

  int bar;
  dynamic_params_client->get_event_param_or(event, "bar", bar, 4);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "bar: %d", bar);

  // Parameter not set on server

  std::vector<std::string> foobar;
  dynamic_params_client->get_event_param_or(event, "foobar", foobar, {"default"});
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
  
/*   
  std::stringstream ss;
  ss << "\nNode names:";
  for (auto & name : remote_nodes) {
    ss << "\n" << name;
  } */

  std::string remote_node_name = "/param_tests";

  // Add Dynamic Reconfigure Client
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node, {remote_node_name});
  dynamic_params_client->add_parameters({"foo", "bar"});
  dynamic_params_client->add_parameters("foobar");
  dynamic_params_client->set_callback(std::bind(event_callback, std::placeholders::_1));

 /*  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node_name);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again..."); 
  } */

/*   RCLCPP_INFO(node->get_logger(), "Listing parameters...");
  // List the details of a few parameters up to a namespace depth of 10.
  auto parameters_and_prefixes = client->list_parameters({}, 10);

  ss.clear();
  ss << "\nParameter names:";
  for (auto & name : parameters_and_prefixes.names) {
    ss << "\n " << name;
  }
  ss << "\nParameter prefixes:";
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    ss << "\n " << prefix;
  }

  RCLCPP_INFO(node->get_logger(), ss.str().c_str()); */
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
