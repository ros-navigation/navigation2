#include <math.h>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_safety_nodes/nav2_safety_node.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(safetyTest, test_safety_node)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("safetyZoneTest");

  node->declare_parameter("safety_polygon", std::string("[]"));
  node->set_parameter(rclcpp::Parameter("safety_polygon", std::string("[[-0.325, -0.325], [-0.325, 0.325], [0.325, 0.325], [0.46, 0.0], [0.325, -0.325]]"));
  
  auto safety_node = std::make_unique<nav2_safety_nodes::SafetyZone>();
  safety_node->on_configure();
  safety_node->on_activate();
  try {
    safety_node->detectPoints(cloud, safety_zone_);
  } catch (...) {
  }
  safety_node->deactivate();
  safety_node->cleanup();
 
  safety_node.reset();
}
