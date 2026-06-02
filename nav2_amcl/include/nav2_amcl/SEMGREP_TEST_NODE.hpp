#ifndef SEMGREP_TEST_NODE_HPP_
#define SEMGREP_TEST_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace semgrep_test {

// Test class to trigger all 17 semgrep patterns
class SemgrepTestNode {
 public:
  explicit SemgrepTestNode()
    : node_(nullptr) {
  }

  void initializeWithPatterns() {
    // ===================================================================
    // PATTERN 1: rclcpp::Node type usage
    // ===================================================================
    rclcpp::Node::SharedPtr raw_node_;
    std::shared_ptr<rclcpp::Node> my_node;

    // ===================================================================
    // PATTERN 2: rclcpp::Client<T> type usage
    // ===================================================================
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr empty_client_;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> trigger_client;

    // ===================================================================
    // PATTERN 3: rclcpp::Service<T> type usage
    // ===================================================================
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr empty_service_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> trigger_service;

    // ===================================================================
    // PATTERN 4: rclcpp::Publisher<T> type usage
    // ===================================================================
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_pub;

    // ===================================================================
    // PATTERN 5: rclcpp::Subscription<T> type usage
    // ===================================================================
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>> path_sub;

    // ===================================================================
    // PATTERN 6: rclcpp_action::Client<T> type usage
    // ===================================================================
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
    std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> action_client;

    // ===================================================================
    // PATTERN 7: rclcpp_action::Server<T> type usage
    // ===================================================================
    rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_server_;
    std::shared_ptr<rclcpp_action::Server<nav2_msgs::action::NavigateToPose>> action_server;
  }

  void testFreeFactoryFunctions() {
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // ===================================================================
    // PATTERN 8: rclcpp::create_client<T> factory call
    // ===================================================================
    auto empty_client = rclcpp::create_client<std_srvs::srv::Empty>(node_);

    // ===================================================================
    // PATTERN 9: rclcpp::create_service<T> factory call
    // ===================================================================
    auto empty_service = rclcpp::create_service<std_srvs::srv::Empty>(
      node_, "empty_service", [](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                 std::shared_ptr<std_srvs::srv::Empty::Response> response) {});

    // ===================================================================
    // PATTERN 10: rclcpp::create_publisher<T> factory call
    // ===================================================================
    auto path_pub = rclcpp::create_publisher<nav_msgs::msg::Path>(
      node_, "path", rclcpp::QoS(10));

    // ===================================================================
    // PATTERN 11: rclcpp::create_subscription<T> factory call
    // ===================================================================
    auto scan_sub = rclcpp::create_subscription<sensor_msgs::msg::LaserScan>(
      node_, "scan", rclcpp::QoS(10),
      [](const sensor_msgs::msg::LaserScan::SharedPtr msg) {});

    // ===================================================================
    // PATTERN 12: rclcpp_action::create_client<T> factory call
    // ===================================================================
    auto nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "navigate_to_pose");

    // ===================================================================
    // PATTERN 13: rclcpp_action::create_server<T> factory call
    // ===================================================================
    auto nav_server = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
      node_, "navigate_to_pose",
      [](const rclcpp_action::GoalUUID & uuid,
         std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>>
           goal_handle) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>>
           goal_handle) {});
  }

  void testNodeMemberFactoryFunctions() {
    // ===================================================================
    // PATTERN 14: rclcpp::Node::create_publisher<T> member call
    // ===================================================================
    auto path_pub = node_->create_publisher<nav_msgs::msg::Path>(
      "path", rclcpp::QoS(10));

    // ===================================================================
    // PATTERN 15: rclcpp::Node::create_subscription<T> member call
    // ===================================================================
    auto scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10),
      [](const sensor_msgs::msg::LaserScan::SharedPtr msg) {});

    // ===================================================================
    // PATTERN 16: rclcpp::Node::create_client<T> member call
    // ===================================================================
    auto empty_client = node_->create_client<std_srvs::srv::Empty>(
      "empty_service");

    // ===================================================================
    // PATTERN 17: rclcpp::Node::create_service<T> member call
    // ===================================================================
    auto empty_service = node_->create_service<std_srvs::srv::Empty>(
      "empty_service", [](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response) {});
    
    // ===================================================================
    // DIRECT PATTERN: rclcpp::Node::create_publisher<T> via pointer-to-member
    // ===================================================================
    auto create_publisher_ptr = &rclcpp::Node::create_publisher<nav_msgs::msg::Path>;
    auto direct_path_pub = ((*node_).*create_publisher_ptr)(
      "direct_path", rclcpp::QoS(10));

    // ===================================================================
    // DIRECT PATTERN: rclcpp::Node::create_subscription<T> via pointer-to-member
    // ===================================================================
    auto create_subscription_ptr = &rclcpp::Node::create_subscription<sensor_msgs::msg::LaserScan>;
    auto direct_scan_sub = ((*node_).*create_subscription_ptr)(
      "direct_scan", rclcpp::QoS(10),
      [](const sensor_msgs::msg::LaserScan::SharedPtr msg) {});

    // ===================================================================
    // DIRECT PATTERN: rclcpp::Node::create_client<T> via pointer-to-member
    // ===================================================================
    auto create_client_ptr = &rclcpp::Node::create_client<std_srvs::srv::Empty>;
    auto direct_empty_client = ((*node_).*create_client_ptr)(
      "direct_empty_service");

    // ===================================================================
    // DIRECT PATTERN: rclcpp::Node::create_service<T> via pointer-to-member
    // ===================================================================
    auto create_service_ptr = &rclcpp::Node::create_service<std_srvs::srv::Empty>;
    auto direct_empty_service = ((*node_).*create_service_ptr)(
      "direct_empty_service", [](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> response) {});
  }

 private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace semgrep_test

#endif  // SEMGREP_TEST_NODE_HPP_
