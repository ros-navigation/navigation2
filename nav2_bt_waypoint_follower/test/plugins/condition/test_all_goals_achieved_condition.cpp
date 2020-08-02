#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_bt_waypoint_follower/plugins/condition/all_goals_achieved_condition.hpp"

class AllGoalsAchievedConditionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("get_next_goal_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("path_updated", false);
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<int>("number_recoveries", 0);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_bt_waypoint_follower::AllGoalsAchievedCondition>(
          name, config);
      };

    factory_->registerBuilder<nav2_bt_waypoint_follower::AllGoalsAchievedCondition>(
      "AllGoalAchieved", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    config_->blackboard->set("current_waypoint_idx", 0);
    config_->blackboard->set("num_waypoints", 3);
    config_->blackboard->set("goal_achieved", false);
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr AllGoalsAchievedConditionTestFixture::node_ = nullptr;
BT::NodeConfiguration * AllGoalsAchievedConditionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> AllGoalsAchievedConditionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> AllGoalsAchievedConditionTestFixture::tree_ = nullptr;

TEST_F(AllGoalsAchievedConditionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <AllGoalAchieved goal_achieved="{goal_achieved}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<bool>("goal_achieved"), false);
}

TEST_F(AllGoalsAchievedConditionTestFixture, test_tick_not_achieved)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <AllGoalAchieved goal_achieved="{goal_achieved}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int64_t>("current_waypoint_idx"), 0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);

  config_->blackboard->set("goal_achieved", true);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);

  config_->blackboard->set("goal_achieved", false);
  config_->blackboard->set("current_waypoint_idx", 2);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(AllGoalsAchievedConditionTestFixture, test_tick_achieved)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <AllGoalAchieved goal_achieved="{goal_achieved}" />
        </BehaviorTree>
      </root>)";

  config_->blackboard->set("goal_achieved", true);
  config_->blackboard->set("current_waypoint_idx", 2);
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int64_t>("current_waypoint_idx"), 2);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
