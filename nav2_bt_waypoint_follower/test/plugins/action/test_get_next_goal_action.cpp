#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_bt_waypoint_follower/plugins/action/get_next_goal_action.hpp"

class GetNextGoalActionTestFixture : public ::testing::Test
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
        return std::make_unique<nav2_bt_waypoint_follower::GetNextGoalAction>(
          name, config);
      };

    factory_->registerBuilder<nav2_bt_waypoint_follower::GetNextGoalAction>("GetNextGoal", builder);
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
    nav2_msgs::action::FollowWaypoints::Goal goal;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.0;
    pose.pose.orientation.w = 1.0;
    goal.poses.push_back(pose);
    pose.pose.position.x = 2.0;
    goal.poses.push_back(pose);
    pose.pose.position.x = 3.0;
    goal.poses.push_back(pose);
    config_->blackboard->set("goals", goal.poses);
    config_->blackboard->set("current_waypoint_idx", 0);
    config_->blackboard->set("num_waypoints", goal.poses.size());
    config_->blackboard->set("goal", goal.poses[0]);
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

rclcpp::Node::SharedPtr GetNextGoalActionTestFixture::node_ = nullptr;
BT::NodeConfiguration * GetNextGoalActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GetNextGoalActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GetNextGoalActionTestFixture::tree_ = nullptr;

TEST_F(GetNextGoalActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GetNextGoal goals="{goals}" goal_achieved="{goal_achieved}" goal="{goal}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  tree_->rootNode()->getInput("goals", goals);
  EXPECT_EQ(goals.size(), 3u);
  EXPECT_EQ(tree_->rootNode()->getInput<bool>("goal_achieved"), false);
}

TEST_F(GetNextGoalActionTestFixture, test_tick_not_update)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GetNextGoal goals="{goals}" goal_achieved="{goal_achieved}" goal="{goal}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int64_t>("current_waypoint_idx"), 0);
  EXPECT_EQ(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("goal").pose.position.x, 1.0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<bool>("goal_achieved"), false);
  EXPECT_EQ(config_->blackboard->get<int64_t>("current_waypoint_idx"), 0);
  EXPECT_EQ(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("goal").pose.position.x, 1.0);
}

TEST_F(GetNextGoalActionTestFixture, test_tick_update)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GetNextGoal goals="{goals}" goal_achieved="{goal_achieved}" goal="{goal}" />
        </BehaviorTree>
      </root>)";

  config_->blackboard->set("goal_achieved", true);
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int64_t>("current_waypoint_idx"), 0);
  EXPECT_EQ(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("goal").pose.position.x, 1.0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<bool>("goal_achieved"), false);
  EXPECT_EQ(config_->blackboard->get<int64_t>("current_waypoint_idx"), 1);
  EXPECT_EQ(config_->blackboard->get<geometry_msgs::msg::PoseStamped>("goal").pose.position.x, 2.0);
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
