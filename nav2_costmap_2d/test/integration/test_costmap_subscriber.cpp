// Copyright (c) 2020 Samsung Research
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;
class TestCostmapSubscriberShould : public ::testing::Test
{
public:
  TestCostmapSubscriberShould()
  : topicName("/costmap"), node(nav2_util::LifecycleNode::make_shared("test_subscriber"))
  {
    dummyCostmapMsgSubscriber = node->create_subscription<nav2_msgs::msg::Costmap>(
      topicName + "_raw", 10,
      std::bind(&TestCostmapSubscriberShould::costmapCallback, this, std::placeholders::_1));

    dummyCostmapUpdateMsgSubscriber =
      node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
      topicName + "_raw_updates", 10, std::bind(
        &TestCostmapSubscriberShould::costmapUpdateCallback, this,
        std::placeholders::_1));
  }

  void SetUp() override
  {
    fullCostmapMsgCount = 0;
    updateCostmapMsgCount = 0;

    costmapSubscriber =
      std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, topicName + "_raw");

    costmapToSend = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
  }

  void TearDown() override
  {
    costmapSubscriber.reset();
    costmapToSend.reset();
  }

  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr)
  {
    this->fullCostmapMsgCount++;
  }
  void costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::SharedPtr)
  {
    this->updateCostmapMsgCount++;
  }

  struct CostmapObservation
  {
    std::uint32_t x;
    std::uint32_t y;
    std::uint8_t cost;
  };

  struct MapChange
  {
    std::vector<CostmapObservation> observations;
    std::uint32_t x0;
    std::uint32_t xn;
    std::uint32_t y0;
    std::uint32_t yn;
  };

/* *INDENT-OFF* */
  const std::vector<MapChange> mapChanges {{{{2, 2, 255}, {2, 3, 255}, {3, 2, 255}}, 2, 4, 2, 4},
                                           {{{7, 7, 255}, {7, 8, 255}}, 7, 8, 7, 9},
                                           {{{2, 2, 0}, {2, 3, 0}, {3, 2, 0}}, 2, 4, 2, 4}};
/* *INDENT-ON* */

protected:
  std::vector<uint8_t> getCurrentCharMapFromSubscriber()
  {
    auto currentSubscriberCostmap = costmapSubscriber->getCostmap();
    return
      std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX());
  }

  std::vector<uint8_t> getCurrentCharMapToSend()
  {
    return std::vector<uint8_t>(
      costmapToSend->getCharMap(),
      costmapToSend->getCharMap() + costmapToSend->getSizeInCellsX() *
      costmapToSend->getSizeInCellsX());
  }

  int fullCostmapMsgCount;
  int updateCostmapMsgCount;
  std::string topicName;

  nav2_util::LifecycleNode::SharedPtr node;
  rclcpp::Logger logger {rclcpp::get_logger("test_costmap_subscriber_should")};

  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmapSubscriber;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmapToSend;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr dummyCostmapMsgSubscriber;
  rclcpp::Subscription<nav2_msgs::msg::CostmapUpdate>::SharedPtr dummyCostmapUpdateMsgSubscriber;
};

TEST_F(TestCostmapSubscriberShould, handleFullCostmapMsgs)
{
  bool always_send_full_costmap = true;

  std::vector<std::vector<std::uint8_t>> expectedCostmaps;
  std::vector<std::vector<std::uint8_t>> recievedCostmaps;

  auto costmapPublisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node, costmapToSend.get(), "", topicName, always_send_full_costmap);
  costmapPublisher->on_activate();

  for (const auto & mapChange : mapChanges) {
    for (const auto & observation : mapChange.observations) {
      costmapToSend->setCost(observation.x, observation.y, observation.cost);
    }

    expectedCostmaps.emplace_back(getCurrentCharMapToSend());

    costmapPublisher->updateBounds(mapChange.x0, mapChange.xn, mapChange.y0, mapChange.yn);
    costmapPublisher->publishCostmap();

    rclcpp::spin_some(node->get_node_base_interface());

    recievedCostmaps.emplace_back(getCurrentCharMapFromSubscriber());
  }

  ASSERT_EQ(fullCostmapMsgCount, mapChanges.size());
  ASSERT_EQ(updateCostmapMsgCount, 0);

  ASSERT_EQ(expectedCostmaps, recievedCostmaps);

  costmapPublisher->on_deactivate();
}

TEST_F(TestCostmapSubscriberShould, handleCostmapUpdateMsgs)
{
  bool always_send_full_costmap = false;

  std::vector<std::vector<std::uint8_t>> expectedCostmaps;
  std::vector<std::vector<std::uint8_t>> recievedCostmaps;

  auto costmapPublisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node, costmapToSend.get(), "", topicName, always_send_full_costmap);
  costmapPublisher->on_activate();

  for (const auto & mapChange : mapChanges) {
    for (const auto & observation : mapChange.observations) {
      costmapToSend->setCost(observation.x, observation.y, observation.cost);
    }

    expectedCostmaps.emplace_back(getCurrentCharMapToSend());

    costmapPublisher->updateBounds(mapChange.x0, mapChange.xn, mapChange.y0, mapChange.yn);
    costmapPublisher->publishCostmap();

    rclcpp::spin_some(node->get_node_base_interface());

    recievedCostmaps.emplace_back(getCurrentCharMapFromSubscriber());
  }

  ASSERT_EQ(fullCostmapMsgCount, 1);
  ASSERT_EQ(updateCostmapMsgCount, mapChanges.size() - 1);

  ASSERT_EQ(expectedCostmaps, recievedCostmaps);

  costmapPublisher->on_deactivate();
}

TEST_F(
  TestCostmapSubscriberShould,
  throwExceptionIfGetCostmapMethodIsCalledBeforeAnyCostmapMsgReceived)
{
  ASSERT_ANY_THROW(costmapSubscriber->getCostmap());
}
<<<<<<< HEAD
=======

TEST(CostmapPublisherShould, publishInitialCostmapForLateSubscribers)
{
  constexpr unsigned int size_x = 10;
  constexpr unsigned int size_y = 10;
  auto node = std::make_shared<nav2::LifecycleNode>("test_late_costmap_subscriber");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    size_x, size_y, 1.0, 0.0, 0.0);
  auto costmap_publisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node, costmap.get(), "map", "/late_costmap", false);
  costmap_publisher->on_activate();

  // Publish before any subscribers exist. The transient-local publishers must retain this
  // initial full costmap for subscribers that join later.
  costmap_publisher->publishCostmap();

  std::promise<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> costmap_promise;
  auto costmap_future = costmap_promise.get_future();
  auto costmap_subscriber = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/late_costmap",
    [&costmap_promise](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      costmap_promise.set_value(msg);
    }, nav2::qos::LatchedSubscriptionQoS());

  std::promise<nav2_msgs::msg::Costmap::ConstSharedPtr> raw_costmap_promise;
  auto raw_costmap_future = raw_costmap_promise.get_future();
  auto raw_costmap_subscriber = node->create_subscription<nav2_msgs::msg::Costmap>(
    "/late_costmap_raw",
    [&raw_costmap_promise](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
      raw_costmap_promise.set_value(msg);
    }, nav2::qos::LatchedSubscriptionQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  ASSERT_EQ(
    executor.spin_until_future_complete(costmap_future, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(
    executor.spin_until_future_complete(raw_costmap_future, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto received_costmap = costmap_future.get();
  EXPECT_EQ(received_costmap->info.width, size_x);
  EXPECT_EQ(received_costmap->info.height, size_y);
  EXPECT_EQ(received_costmap->data.size(), size_x * size_y);

  const auto received_raw_costmap = raw_costmap_future.get();
  EXPECT_EQ(received_raw_costmap->metadata.size_x, size_x);
  EXPECT_EQ(received_raw_costmap->metadata.size_y, size_y);
  EXPECT_EQ(received_raw_costmap->data.size(), size_x * size_y);

  costmap_publisher->on_deactivate();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
>>>>>>> 704bcc71 (Fix costmap publication for late subscribers (#6324))
