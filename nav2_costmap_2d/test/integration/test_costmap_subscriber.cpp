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
  : subscriberNode(nav2_util::LifecycleNode::make_shared("test_subscriber"))
  {
  }

  void SetUp() override
  {
    fullCostmapMsgCount = 0;
    updateCostmapMsgCount = 0;

    topicName = "/costmap";
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
  nav2_util::LifecycleNode::SharedPtr subscriberNode;
  rclcpp::Logger logger {rclcpp::get_logger("test_costmap_subscriber_should")};
};

TEST_F(TestCostmapSubscriberShould, handleCostmapUpdateMsgs)
{
  // std::vector<std::vector<std::uint8_t>> expectedCostmaps(4, std::vector<std::uint8_t>(100, 0)),
  // recievedCostmaps(4, std::vector<std::uint8_t>(100, 0));
  auto dummyCostmapMsgSubscriber = subscriberNode->create_subscription<nav2_msgs::msg::Costmap>(
    topicName + "_raw", 10,
    std::bind(&TestCostmapSubscriberShould::costmapCallback, this, std::placeholders::_1));

  auto dummyCostmapUpdateMsgSubscriber =
    subscriberNode->create_subscription<nav2_msgs::msg::CostmapUpdate>(
    topicName + "_raw_updates", 10, std::bind(
      &TestCostmapSubscriberShould::costmapUpdateCallback, this,
      std::placeholders::_1));


  auto costmapSubscriber =
    std::make_shared<nav2_costmap_2d::CostmapSubscriber>(subscriberNode, topicName + "_raw");

  bool always_send_full_costmap = false;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
  nav2_util::LifecycleNode::SharedPtr publisherNode = nav2_util::LifecycleNode::make_shared(
    "test_publisher");
  auto costmapPublisher = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
    publisherNode, costmap.get(), "", topicName, always_send_full_costmap);
  costmapPublisher->on_activate();


  costmapPublisher->publishCostmap();
  rclcpp::spin_some(subscriberNode->get_node_base_interface());
  rclcpp::Rate(0.1).sleep();


  auto currentSubscriberCostmap = costmapSubscriber->getCostmap();

  RCLCPP_WARN_STREAM(
    logger,
    "CURRENT COSTMAP SIZE " << currentSubscriberCostmap->getSizeInCellsX());

  costmap->setCost(2, 2, 255);
  costmap->setCost(2, 3, 255);
  costmap->setCost(3, 2, 255);
  costmapPublisher->updateBounds(2, 4, 2, 4);
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(subscriberNode->get_node_base_interface());
  rclcpp::Rate(0.1).sleep();
  auto currentSubscriberCostmap2 = costmapSubscriber->getCostmap();


  costmap->setCost(7, 7, 255);
  costmap->setCost(7, 8, 255);
  costmapPublisher->updateBounds(7, 8, 7, 9);
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(subscriberNode->get_node_base_interface());
  rclcpp::Rate(0.1).sleep();


  costmap->setCost(2, 2, 0);
  costmap->setCost(2, 3, 0);
  costmap->setCost(3, 2, 0);
  costmapPublisher->updateBounds(2, 4, 2, 4);
  costmapPublisher->publishCostmap();
  rclcpp::spin_some(subscriberNode->get_node_base_interface());
  rclcpp::Rate(0.1).sleep();

  ASSERT_EQ(fullCostmapMsgCount, 1);
  ASSERT_EQ(updateCostmapMsgCount, 3);

  costmapPublisher->on_deactivate();
}

TEST_F(TestCostmapSubscriberShould, TestIfNotCollapsing)
{
  ASSERT_EQ(1, 1);
}

// TEST_F(TestCostmapSubscriberShould, handleFullCostmapMsgs)
// {
//   // std::vector<std::vector<std::uint8_t>> expectedCostmaps,recievedCostmaps;

//   // bool always_send_full_costmap = true;

//   // auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);

//   // nav2_util::LifecycleNode::SharedPtr publisherNode = nav2_util::LifecycleNode::make_shared(
//   //   "test_publisher_2");

//   // auto costmapPublisher = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
//   //   publisherNode, costmap.get(), "", topicName, always_send_full_costmap);

//   // costmapPublisher->on_activate();

//   // costmapPublisher->publishCostmap();
//   // rclcpp::Rate(0.1).sleep();

//   // costmapPublisher->publishCostmap();
//   // rclcpp::Rate(0.1).sleep();

//   // costmapPublisher->publishCostmap();
//   // rclcpp::Rate(0.1).sleep();

//   // ASSERT_EQ(fullCostmapMsgCount, 3);
//   // ASSERT_EQ(updateCostmapMsgCount, 0);

//   // costmapPublisher->on_deactivate();

//   ASSERT_TRUE(false);
// }
