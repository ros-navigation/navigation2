#include <memory>
#include <string>
#include <algorithm>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "../testing_helper.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/plugin_container_layer.hpp"

using std::begin;
using std::end;
using std::for_each;
using std::all_of;
using std::none_of;
using std::pair;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const string & name)
  : nav2_util::LifecycleNode(name)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    node_ = std::make_shared<TestLifecycleNode>("plugin_container_test_node");
    node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    node_->declare_parameter("unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  }

  ~TestNode() {}

protected:
  std::shared_ptr<TestLifecycleNode> node_;
};

/*
 * For reference, the static map looks like this:
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0 254 254 254   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   upper left is 0,0, lower right is 9,9
 */

/**
 * Test if combining layers different plugin container layers works
 */

TEST_F(TestNode, testAddingLayers) {
  tf2_ros::Buffer tf(node_->get_clock());

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);
  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;

  //Add one plugin container layer
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  //Add a second plugin container layer
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  //Initialize an obstacle layer and add it to the first plugin container layer
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_a = std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_a->addPlugin(olayer_a, "obstacles");

  //Initialize an obstacle layer and add it to the second plugin container layer
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b = std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "obstacles");

  addObservation(olayer_a, 5.0, 5.0);
  addObservation(olayer_b, 3.0, 8.0);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  //We expect that the two plugin container layers have combined to give us two lethal points
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 2);
}
