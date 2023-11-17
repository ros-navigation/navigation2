#include <gtest/gtest.h>

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
  }

  void SetUp() override
  {
    fullCostmapMsgCount = 0;
    updateCostmapMsgCount = 0;
  }

  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr)
  {
    this->fullCostmapMsgCount++;
  }
  void costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::SharedPtr)
  {
    this->updateCostmapMsgCount++;
  }

protected:
  int fullCostmapMsgCount;
  int updateCostmapMsgCount;
  std::string topicName;

  std::thread subscriberThread;
  std::weak_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  nav2_util::LifecycleNode::SharedPtr node;
  rclcpp::Logger logger {rclcpp::get_logger("test_costmap_subscriber_should")};
};

TEST_F(TestCostmapSubscriberShould, handleFullCostmapMsgs)
{
  bool always_send_full_costmap = true;

  std::vector<std::vector<std::uint8_t>> expectedCostmaps;
  std::vector<std::vector<std::uint8_t>> recievedCostmaps;

  auto dummyCostmapMsgSubscriber = node->create_subscription<nav2_msgs::msg::Costmap>(
    topicName + "_raw", 10,
    std::bind(&TestCostmapSubscriberShould::costmapCallback, this, std::placeholders::_1));

  auto dummyCostmapUpdateMsgSubscriber =
    node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
    topicName + "_raw_updates", 10, std::bind(
      &TestCostmapSubscriberShould::costmapUpdateCallback, this,
      std::placeholders::_1));

  auto costmapSubscriber =
    std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, topicName + "_raw");

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);

  auto costmapPublisher = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
    node, costmap.get(), "", topicName, always_send_full_costmap);
  costmapPublisher->on_activate();

  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());

  auto currentSubscriberCostmap = costmapSubscriber->getCostmap();
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));

  costmap->setCost(2, 2, 255);
  costmap->setCost(2, 3, 255);
  costmap->setCost(3, 2, 255);
  costmapPublisher->updateBounds(2, 4, 2, 4);

  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));


  costmap->setCost(7, 7, 255);
  costmap->setCost(7, 8, 255);
  costmapPublisher->updateBounds(7, 8, 7, 9);
  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));


  costmap->setCost(2, 2, 0);
  costmap->setCost(2, 3, 0);
  costmap->setCost(3, 2, 0);
  costmapPublisher->updateBounds(2, 4, 2, 4);
  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));

  ASSERT_EQ(fullCostmapMsgCount, 1);
  ASSERT_EQ(updateCostmapMsgCount, 3);

  ASSERT_EQ(expectedCostmaps, recievedCostmaps);

  costmapPublisher->on_deactivate();
}

TEST_F(TestCostmapSubscriberShould, handleCostmapUpdateMsgs)
{
  bool always_send_full_costmap = false;

  std::vector<std::vector<std::uint8_t>> expectedCostmaps;
  std::vector<std::vector<std::uint8_t>> recievedCostmaps;

  auto dummyCostmapMsgSubscriber = node->create_subscription<nav2_msgs::msg::Costmap>(
    topicName + "_raw", 10,
    std::bind(&TestCostmapSubscriberShould::costmapCallback, this, std::placeholders::_1));

  auto dummyCostmapUpdateMsgSubscriber =
    node->create_subscription<nav2_msgs::msg::CostmapUpdate>(
    topicName + "_raw_updates", 10, std::bind(
      &TestCostmapSubscriberShould::costmapUpdateCallback, this,
      std::placeholders::_1));


  auto costmapSubscriber =
    std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, topicName + "_raw");

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);

  auto costmapPublisher = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
    node, costmap.get(), "", topicName, always_send_full_costmap);
  costmapPublisher->on_activate();

  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());

  auto currentSubscriberCostmap = costmapSubscriber->getCostmap();
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));

  costmap->setCost(2, 2, 255);
  costmap->setCost(2, 3, 255);
  costmap->setCost(3, 2, 255);
  costmapPublisher->updateBounds(2, 4, 2, 4);

  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));


  costmap->setCost(7, 7, 255);
  costmap->setCost(7, 8, 255);
  costmapPublisher->updateBounds(7, 8, 7, 9);
  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));


  costmap->setCost(2, 2, 0);
  costmap->setCost(2, 3, 0);
  costmap->setCost(3, 2, 0);
  costmapPublisher->updateBounds(2, 4, 2, 4);
  expectedCostmaps.emplace_back(
    std::vector<uint8_t>(
      costmap->getCharMap(),
      costmap->getCharMap() + costmap->getSizeInCellsX() * costmap->getSizeInCellsX()));
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(node->get_node_base_interface());
  recievedCostmaps.emplace_back(
    std::vector<uint8_t>(
      currentSubscriberCostmap->getCharMap(),
      currentSubscriberCostmap->getCharMap() + currentSubscriberCostmap->getSizeInCellsX() *
      currentSubscriberCostmap->getSizeInCellsX()));

  ASSERT_EQ(fullCostmapMsgCount, 1);
  ASSERT_EQ(updateCostmapMsgCount, 3);

  ASSERT_EQ(expectedCostmaps, recievedCostmaps);

  costmapPublisher->on_deactivate();
}

TEST_F(
  TestCostmapSubscriberShould,
  throwExceptionIfGetCostmapMethodIsCalledBeforeAnyCostmapMsgReceived)
{
  auto costmapSubscriber =
    std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, topicName + "_raw");

  ASSERT_ANY_THROW(costmapSubscriber->getCostmap());
}
