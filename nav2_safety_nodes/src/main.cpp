#include <memory>

#include "nav2_safety_nodes/nav2_safety_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // auto node = std::make_shared<nav2_safety_nodes::SafetyZone>();
  rclcpp::spin(std::make_shared<nav2_safety_nodes::SafetyZone>());
  rclcpp::shutdown();

  return 0;
}