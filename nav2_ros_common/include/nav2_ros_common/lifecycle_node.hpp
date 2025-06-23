// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROS_COMMON__LIFECYCLE_NODE_HPP_
#define NAV2_ROS_COMMON__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <chrono>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav2_ros_common/node_thread.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bondcpp/bond.hpp"
#include "bond/msg/constants.hpp"
#include "nav2_ros_common/interface_factories.hpp"

namespace nav2
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;  // NOLINT

/**
 * @class nav2::LifecycleNode
 * @brief A lifecycle node wrapper to enable common Nav2 needs such as manipulating parameters
 */
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using SharedPtr = std::shared_ptr<nav2::LifecycleNode>;
  using WeakPtr = std::weak_ptr<nav2::LifecycleNode>;
  using SharedConstPointer = std::shared_ptr<const nav2::LifecycleNode>;

  /**
   * @brief A lifecycle node constructor
   * @param node_name Name for the node
   * @param namespace Namespace for the node, if any
   * @param options Node options
   */
  LifecycleNode(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
  {
    // server side never times out from lifecycle manager
    this->declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
    this->set_parameter(
      rclcpp::Parameter(
        bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

    nav2::declare_parameter_if_not_declared(
      this, "bond_heartbeat_period", rclcpp::ParameterValue(0.1));
    this->get_parameter("bond_heartbeat_period", bond_heartbeat_period);

    bool autostart_node = false;
    nav2::declare_parameter_if_not_declared(
      this, "autostart_node", rclcpp::ParameterValue(false));
    this->get_parameter("autostart_node", autostart_node);
    if (autostart_node) {
      autostart();
    }

    printLifecycleNodeNotification();

    register_rcl_preshutdown_callback();
  }

  /**
   * @brief A lifecycle node constructor with no namespace
   * @param node_name Name for the node
   * @param options Node options
   */
  LifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2::LifecycleNode(node_name, "", options)
  {}

  virtual ~LifecycleNode()
  {
    RCLCPP_INFO(get_logger(), "Destroying");

    runCleanups();

    if (rcl_preshutdown_cb_handle_) {
      rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();
      context->remove_pre_shutdown_callback(*(rcl_preshutdown_cb_handle_.get()));
      rcl_preshutdown_cb_handle_.reset();
    }
  }

  /**
   * @brief Create a subscription to a topic using Nav2 QoS profiles and SubscriptionOptions
   * @param topic_name Name of topic
   * @param callback Callback function to handle incoming messages
   * @param qos QoS settings for the subscription (default is nav2::qos::StandardTopicQoS())
   * @param callback_group The callback group to use (if provided)
   * @return A shared pointer to the created nav2::Subscription
   */
  template<
    typename MessageT,
    typename CallbackT>
  std::shared_ptr<nav2::Subscription<MessageT>>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
    const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
  {
    return nav2::interfaces::create_subscription<MessageT>(
      shared_from_this(), topic_name,
      std::forward<CallbackT>(callback), qos, callback_group);
  }

  /**
   * @brief Create a publisher to a topic using Nav2 QoS profiles and PublisherOptions
   * @param topic_name Name of topic
   * @param qos QoS settings for the publisher (default is nav2::qos::StandardTopicQoS())
   * @param callback_group The callback group to use (if provided)
   * @return A shared pointer to the created nav2::Publisher
   */
  template<typename MessageT>
  typename nav2::Publisher<MessageT>::SharedPtr
  create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos = nav2::qos::StandardTopicQoS(),
    const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
  {
    auto pub = nav2::interfaces::create_publisher<MessageT>(
      shared_from_this(), topic_name, qos, callback_group);
    this->add_managed_entity(pub);
    return pub;
  }

  /**
   * @brief Create a ServiceClient to interface with a service
   * @param service_name Name of service
   * @param use_internal_executor Whether to use the internal executor (default is false)
   * @return A shared pointer to the created nav2::ServiceClient
   */
  template<typename ServiceT>
  typename nav2::ServiceClient<ServiceT>::SharedPtr
  create_client(
    const std::string & service_name,
    bool use_internal_executor = false)
  {
    return nav2::interfaces::create_client<ServiceT>(
      shared_from_this(), service_name, use_internal_executor);
  }

  /**
   * @brief Create a ServiceServer to host with a service
   * @param service_name Name of service
   * @param callback Callback function to handle service requests
   * @param callback_group The callback group to use (if provided)
   * @return A shared pointer to the created nav2::ServiceServer
   */
  template<typename ServiceT>
  typename nav2::ServiceServer<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    typename nav2::ServiceServer<ServiceT>::CallbackType cb,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  {
    return nav2::interfaces::create_service<ServiceT>(
      shared_from_this(), service_name, cb, callback_group);
  }

  /**
   * @brief Create a SimpleActionServer to host with an action
   * @param action_name Name of action
   * @param execute_callback Callback function to handle action execution
   * @param completion_callback Callback function to handle action completion (optional)
   * @param server_timeout Timeout for the action server (default is 500ms)
   * @param spin_thread Whether to spin with a dedicated thread internally (default is false)
   * @param realtime Whether the action server's worker thread should have elevated
   * @return A shared pointer to the created nav2::SimpleActionServer
   */
  template<typename ActionT>
  typename nav2::SimpleActionServer<ActionT>::SharedPtr
  create_action_server(
    const std::string & action_name,
    typename nav2::SimpleActionServer<ActionT>::ExecuteCallback execute_callback,
    typename nav2::SimpleActionServer<ActionT>::CompletionCallback compl_cb = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
    bool spin_thread = false,
    const bool realtime = false)
  {
    return nav2::interfaces::create_action_server<ActionT>(
      shared_from_this(), action_name, execute_callback,
      compl_cb, server_timeout, spin_thread, realtime);
  }

  /**
   * @brief Create a ActionClient to call an action using
   * @param action_name Name of action
   * @param callback_group The callback group to use (if provided)
   * @return A shared pointer to the created nav2::ActionClient
   */
  template<typename ActionT>
  typename nav2::ActionClient<ActionT>::SharedPtr
  create_action_client(
    const std::string & action_name,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  {
    return nav2::interfaces::create_action_client<ActionT>(
      shared_from_this(), action_name, callback_group);
  }

  /**
   * @brief Get a shared pointer of this
   */
  nav2::LifecycleNode::SharedPtr shared_from_this()
  {
    return std::static_pointer_cast<nav2::LifecycleNode>(
      rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  /**
   * @brief Get a shared pointer of this
   */
  nav2::LifecycleNode::WeakPtr weak_from_this()
  {
    return std::static_pointer_cast<nav2::LifecycleNode>(
      rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  /**
   * @brief Abstracted on_error state transition callback, since unimplemented as of 2020
   * in the managed ROS2 node state machine
   * @param state State prior to error transition
   * @return Return type for success or failed transition to error state
   */
  nav2::CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_FATAL(
      get_logger(),
      "Lifecycle node %s does not have error state implemented", get_name());
    return nav2::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Automatically configure and active the node
   */
  void autostart()
  {
    using lifecycle_msgs::msg::State;
    autostart_timer_ = this->create_wall_timer(
      0s,
      [this]() -> void {
        autostart_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Auto-starting node: %s", this->get_name());
        if (configure().id() != State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_ERROR(
            get_logger(), "Auto-starting node %s failed to configure!", this->get_name());
          return;
        }
        if (activate().id() != State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_ERROR(
            get_logger(), "Auto-starting node %s failed to activate!", this->get_name());
        }
      });
  }

  /**
   * @brief Perform preshutdown activities before our Context is shutdown.
   * Note that this is related to our Context's shutdown sequence, not the
   * lifecycle node state machine.
   */
  virtual void on_rcl_preshutdown()
  {
    RCLCPP_INFO(
      get_logger(), "Running Nav2 LifecycleNode rcl preshutdown (%s)",
      this->get_name());

    runCleanups();

    destroyBond();
  }

  /**
   * @brief Create bond connection to lifecycle manager
   */
  void createBond()
  {
    if (bond_heartbeat_period > 0.0) {
      RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", this->get_name());

      bond_ = std::make_unique<bond::Bond>(
        std::string("bond"),
        this->get_name(),
        shared_from_this());

      bond_->setHeartbeatPeriod(bond_heartbeat_period);
      bond_->setHeartbeatTimeout(4.0);
      bond_->start();
    }
  }

  /**
   * @brief Destroy bond connection to lifecycle manager
   */
  void destroyBond()
  {
    if (bond_heartbeat_period > 0.0) {
      RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", this->get_name());

      if (bond_) {
        bond_.reset();
      }
    }
  }

protected:
  /**
   * @brief Print notifications for lifecycle node
   */
  void printLifecycleNodeNotification()
  {
    RCLCPP_INFO(
      get_logger(),
      "\n\t%s lifecycle node launched. \n"
      "\tWaiting on external lifecycle transitions to activate\n"
      "\tSee https://design.ros2.org/articles/node_lifecycle.html for more information.",
      get_name());
  }

  /**
   * Register our preshutdown callback for this Node's rcl Context.
   * The callback fires before this Node's Context is shutdown.
   * Note this is not directly related to the lifecycle state machine.
   */
  void register_rcl_preshutdown_callback()
  {
    rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

    rcl_preshutdown_cb_handle_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
      context->add_pre_shutdown_callback(
        std::bind(&LifecycleNode::on_rcl_preshutdown, this))
    );
  }

  /**
   * Run some common cleanup steps shared between rcl preshutdown and destruction.
   */
  void runCleanups()
  {
    /*
    * In case this lifecycle node wasn't properly shut down, do it here.
    * We will give the user some ability to clean up properly here, but it's
    * best effort; i.e. we aren't trying to account for all possible states.
    */
    if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      this->deactivate();
    }

    if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      this->cleanup();
    }
  }

  // Connection to tell that server is still up
  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> rcl_preshutdown_cb_handle_{nullptr};
  std::unique_ptr<bond::Bond> bond_{nullptr};
  double bond_heartbeat_period{0.1};
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__LIFECYCLE_NODE_HPP_
