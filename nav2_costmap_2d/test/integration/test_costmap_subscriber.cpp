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

    subscriberThread = std::thread(
      [this]()
      {
        auto localExecutor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor = localExecutor;


        localExecutor->add_node(subscriberNode->get_node_base_interface());

        auto costmap_sub = subscriberNode->create_subscription<nav2_msgs::msg::Costmap>(
          topicName + "_raw", 10,
          std::bind(&TestCostmapSubscriberShould::costmapCallback, this, std::placeholders::_1));

        auto costmap_update_sub = subscriberNode->create_subscription<nav2_msgs::msg::CostmapUpdate>(
          topicName + "_raw_updates", 10, std::bind(
            &TestCostmapSubscriberShould::costmapUpdateCallback, this,
            std::placeholders::_1));

        // costmapSubscriber =
        // std::make_shared<nav2_costmap_2d::CostmapSubscriber>(subscriberNode, topicName + "_raw");

        localExecutor->spin();
      }
    );
  }

  void TearDown() override
  {
    if (std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec = executor.lock()) {
      exec->cancel();
    }
    subscriberThread.join();
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

  std::weak_ptr<nav2_costmap_2d::CostmapSubscriber> costmapSubscriber;

  std::thread subscriberThread;
  std::weak_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  nav2_util::LifecycleNode::SharedPtr subscriberNode;

  rclcpp::Logger logger {rclcpp::get_logger("test_costmap_subscriber_should")};
};

TEST_F(TestCostmapSubscriberShould, handleFullCostmapMsgs)
{
  std::vector<std::vector<std::uint8_t>> expectedCostmaps, recievedCostmaps;

  bool always_send_full_costmap = true;

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);

  nav2_util::LifecycleNode::SharedPtr publisherNode = nav2_util::LifecycleNode::make_shared(
    "test_publisher");

  auto costmapPublisher = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
    publisherNode, costmap.get(), "", topicName, always_send_full_costmap);

  costmapPublisher->on_activate();

  costmapPublisher->publishCostmap();
  rclcpp::Rate(0.1).sleep();

  costmapPublisher->publishCostmap();
  rclcpp::Rate(0.1).sleep();

  costmapPublisher->publishCostmap();
  rclcpp::Rate(0.1).sleep();

  ASSERT_EQ(fullCostmapMsgCount, 3);
  ASSERT_EQ(updateCostmapMsgCount, 0);

  costmapPublisher->on_deactivate();
}

// TEST_F(TestCostmapSubscriberShould, handleCostmapUpdateMsgs)
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
