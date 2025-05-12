#include "rclcpp/rclcpp.hpp"
#include "nav2_toolkit/pose_saver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_toolkit::PoseSaverNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
