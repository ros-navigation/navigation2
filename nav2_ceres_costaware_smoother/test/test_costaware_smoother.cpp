#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <thread>
#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_ceres_costaware_smoother/ceres_costaware_smoother.hpp"

using namespace nav2_ceres_costaware_smoother;

class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  DummyCostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name)
  : CostmapSubscriber(node, topic_name)
  {
    auto costmap = std::make_shared<nav2_msgs::msg::Costmap>();
    costmap->metadata.size_x = 100;
    costmap->metadata.size_y = 100;
    costmap->metadata.resolution = 0.1;
    costmap->metadata.origin.position.x = -5.0;
    costmap->metadata.origin.position.y = -5.0;

    costmap->data.resize(costmap->metadata.size_x * costmap->metadata.size_y, 0);
    for (unsigned int i = 0; i < costmap->metadata.size_y; ++i) {
      for (unsigned int j = 20; j < 40; ++j) {
        costmap->data[i * costmap->metadata.size_x + j] = 254;
      }
    }

    setCostmap(costmap);
  }

  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    costmap_msg_ = msg;
    costmap_received_ = true;
  }
};

class SmootherTest : public ::testing::Test
{
protected:
  SmootherTest() {SetUp();}
  ~SmootherTest() {}

  void SetUp()
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "LifecycleRecoveryTestNode", rclcpp::NodeOptions());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    costmap_sub_ =
      std::make_shared<DummyCostmapSubscriber>(
      node_lifecycle_, "costmap_topic");

    smoother_ = std::make_shared<CeresCostawareSmoother>();
    smoother_->configure(
      node_lifecycle_, "Ceres smoother", tf_buffer_, costmap_sub_,
      std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>());

    smoother_->activate();
  }

  void TearDown() override
  {
    smoother_->deactivate();
    smoother_->cleanup();
  }

  bool smoothPath(const std::vector<Eigen::Vector3d> &, std::vector<Eigen::Vector3d> &)
  {

    return true;
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<nav2_ceres_costaware_smoother::CeresCostawareSmoother> smoother_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<DummyCostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
};

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
