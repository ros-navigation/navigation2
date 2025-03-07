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

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

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
    dummyCostmapMsgSubscriber = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topicName, 10,
      std::bind(&TestCostmapSubscriberShould::costmapCallback, this, std::placeholders::_1));

    dummyCostmapRawMsgSubscriber = node->create_subscription<nav2_msgs::msg::Costmap>(
      topicName + "_raw", 10,
      std::bind(&TestCostmapSubscriberShould::costmapRawCallback, this, std::placeholders::_1));

    dummyCostmapUpdateMsgSubscriber = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      topicName + "_updates", 10,
      std::bind(&TestCostmapSubscriberShould::costmapUpdateCallback, this, std::placeholders::_1));

    dummyCostmapRawUpdateMsgSubscriber =
      node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
      topicName + "_raw_updates", 10, std::bind(
        &TestCostmapSubscriberShould::costmapRawUpdateCallback, this,
        std::placeholders::_1));
  }

  void SetUp() override
  {
    fullCostmapMsgCount = 0;
    fullCostmapRawMsgCount = 0;
    updateCostmapMsgCount = 0;
    updateCostmapRawMsgCount = 0;

    costmapSubscriber =
      std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, topicName + "_raw");

    costmapToSend = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
    if (cost_translation_table_ == NULL) {
      cost_translation_table_ = new char[256];

      // special values:
      cost_translation_table_[0] = 0;  // NO obstacle
      cost_translation_table_[253] = 99;  // INSCRIBED obstacle
      cost_translation_table_[254] = 100;  // LETHAL obstacle
      cost_translation_table_[255] = -1;  // UNKNOWN

      // regular cost values scale the range 1 to 252 (inclusive) to fit
      // into 1 to 98 (inclusive).
      for (int i = 1; i < 253; i++) {
        cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
      }
    }
  }

  void TearDown() override
  {
    costmapSubscriber.reset();
    costmapToSend.reset();
  }
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    this->fullCostmapMsgCount++;
    std::vector<uint8_t> data;
    for (unsigned int i = 0; i < msg->data.size(); i++) {
      data.push_back(msg->data[i]);
    }
    receivedGrids.push_back(data);
  }
  void costmapRawCallback(const nav2_msgs::msg::Costmap::SharedPtr)
  {
    this->fullCostmapRawMsgCount++;
  }
  void costmapUpdateCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr update_msg)
  {
    this->updateCostmapMsgCount++;
    std::vector<uint8_t> data;
    for(unsigned int i = 0; i < update_msg->data.size(); i++) {
      data.push_back(update_msg->data[i]);
    }
    receivedGrids.push_back(data);
  }
  void costmapRawUpdateCallback(const nav2_msgs::msg::CostmapUpdate::SharedPtr)
  {
    this->updateCostmapRawMsgCount++;
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
      currentSubscriberCostmap->getSizeInCellsY());
  }

  std::vector<uint8_t> getUpdatedCharMapFromSubscriber(
    std::uint32_t x0, std::uint32_t xn,
    std::uint32_t y0, std::uint32_t yn)
  {
    auto currentSubscriberCostmap = costmapSubscriber->getCostmap();
    std::vector<uint8_t> updatedCostmap;
    for (std::uint32_t y = y0; y < yn; y++) {
      for (std::uint32_t x = x0; x < xn; x++) {
        updatedCostmap.push_back(currentSubscriberCostmap->getCost(x, y));
      }
    }
    return updatedCostmap;
  }

  std::vector<uint8_t> getCurrentCharMapToSend()
  {
    return std::vector<uint8_t>(
      costmapToSend->getCharMap(),
      costmapToSend->getCharMap() + costmapToSend->getSizeInCellsX() *
      costmapToSend->getSizeInCellsY());
  }

  std::vector<uint8_t> getUpdatedCostmapToSend(
    std::uint32_t x0, std::uint32_t xn,
    std::uint32_t y0, std::uint32_t yn)
  {
    std::vector<uint8_t> updatedCostmap;
    for (std::uint32_t y = y0; y < yn; y++) {
      for (std::uint32_t x = x0; x < xn; x++) {
        updatedCostmap.push_back(costmapToSend->getCost(x, y));
      }
    }
    return updatedCostmap;
  }

  int fullCostmapMsgCount;
  int fullCostmapRawMsgCount;
  int updateCostmapMsgCount;
  int updateCostmapRawMsgCount;
  std::string topicName;
  char * cost_translation_table_ = NULL;

  nav2_util::LifecycleNode::SharedPtr node;
  rclcpp::Logger logger {rclcpp::get_logger("test_costmap_subscriber_should")};

  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmapSubscriber;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmapToSend;
  std::vector<std::vector<uint8_t>> receivedGrids;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dummyCostmapMsgSubscriber;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr dummyCostmapRawMsgSubscriber;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
    dummyCostmapUpdateMsgSubscriber;
  rclcpp::Subscription<nav2_msgs::msg::CostmapUpdate>::SharedPtr dummyCostmapRawUpdateMsgSubscriber;
};

TEST_F(TestCostmapSubscriberShould, handleFullCostmapMsgs)
{
  bool always_send_full_costmap = true;

  std::vector<std::vector<std::uint8_t>> expectedCostmaps;
  std::vector<std::vector<std::uint8_t>> expectedGrids;
  std::vector<std::vector<std::uint8_t>> receivedCostmaps;

  auto costmapPublisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node, costmapToSend.get(), "", topicName, always_send_full_costmap);
  costmapPublisher->on_activate();

  for (const auto & mapChange : mapChanges) {
    for (const auto & observation : mapChange.observations) {
      costmapToSend->setCost(observation.x, observation.y, observation.cost);
    }
    std::vector<std::uint8_t> data = getCurrentCharMapToSend();
    expectedCostmaps.emplace_back(data);
    // Grid is expected to be translated from cost translation table
    std::vector<std::uint8_t> grid;
    for (unsigned int i = 0; i < data.size(); i++) {
      grid.push_back(cost_translation_table_[data[i]]);
    }
    expectedGrids.emplace_back(grid);
    costmapPublisher->updateBounds(mapChange.x0, mapChange.xn, mapChange.y0, mapChange.yn);
    costmapPublisher->publishCostmap();
    rclcpp::spin_some(node->get_node_base_interface());
    receivedCostmaps.emplace_back(getCurrentCharMapFromSubscriber());
  }

  ASSERT_EQ(fullCostmapMsgCount, mapChanges.size());
  ASSERT_EQ(fullCostmapRawMsgCount, mapChanges.size());
  ASSERT_EQ(updateCostmapMsgCount, 0);
  ASSERT_EQ(updateCostmapRawMsgCount, 0);

  ASSERT_EQ(expectedCostmaps, receivedCostmaps);
  ASSERT_EQ(expectedGrids, receivedGrids);

  costmapPublisher->on_deactivate();
}

TEST_F(TestCostmapSubscriberShould, handleCostmapUpdateMsgs)
{
  bool always_send_full_costmap = false;

  std::vector<std::vector<std::uint8_t>> expectedCostmaps;
  std::vector<std::vector<std::uint8_t>> expectedGrids;
  std::vector<std::vector<std::uint8_t>> receivedCostmaps;

  auto costmapPublisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node, costmapToSend.get(), "", topicName, always_send_full_costmap);
  costmapPublisher->on_activate();
  std::uint32_t x0 = 0;
  std::uint32_t xn = costmapToSend->getSizeInCellsX();
  std::uint32_t y0 = 0;
  std::uint32_t yn = costmapToSend->getSizeInCellsY();
  bool first_iteration = true;

  for (const auto & mapChange : mapChanges) {
    for (const auto & observation : mapChange.observations) {
      costmapToSend->setCost(observation.x, observation.y, observation.cost);
    }
    // Publish full costmap for the first iteration
    if(!first_iteration) {
      x0 = mapChange.x0;
      xn = mapChange.xn;
      y0 = mapChange.y0;
      yn = mapChange.yn;
    }
    std::vector<std::uint8_t> data = getUpdatedCostmapToSend(x0, xn, y0, yn);
    expectedCostmaps.emplace_back(data);
    std::vector<std::uint8_t> grid;
    for (unsigned int i = 0; i < data.size(); i++) {
      grid.push_back(cost_translation_table_[data[i]]);
    }
    expectedGrids.emplace_back(grid);
    costmapPublisher->updateBounds(mapChange.x0, mapChange.xn, mapChange.y0, mapChange.yn);
    costmapPublisher->publishCostmap();
    rclcpp::spin_some(node->get_node_base_interface());
    receivedCostmaps.emplace_back(getUpdatedCharMapFromSubscriber(x0, xn, y0, yn));
    first_iteration = false;
  }

  ASSERT_EQ(fullCostmapMsgCount, 1);
  ASSERT_EQ(fullCostmapRawMsgCount, 1);
  ASSERT_EQ(updateCostmapMsgCount, mapChanges.size() - 1);
  ASSERT_EQ(updateCostmapRawMsgCount, mapChanges.size() - 1);

  ASSERT_EQ(expectedCostmaps, receivedCostmaps);
  ASSERT_EQ(expectedGrids, receivedGrids);

  costmapPublisher->on_deactivate();
}

TEST_F(
  TestCostmapSubscriberShould,
  throwExceptionIfGetCostmapMethodIsCalledBeforeAnyCostmapMsgReceived)
{
  ASSERT_ANY_THROW(costmapSubscriber->getCostmap());
}
