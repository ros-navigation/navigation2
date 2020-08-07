//
// Created by shivam on 10/3/20.
//

#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_map_server/map_saver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("map_saver_server");
  auto service_node = std::make_shared<nav2_map_server::MapSaver<nav_msgs::msg::OccupancyGrid>>();
  rclcpp::spin(service_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}