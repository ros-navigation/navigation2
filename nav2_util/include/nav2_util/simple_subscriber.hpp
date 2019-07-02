// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_CLIENT_UTIL__SIMPLE_SUBSCRIBER_HPP_
#define NAV2_CLIENT_UTIL__SIMPLE_SUBSCRIBER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_util
{

template<class SubscriberT>
class SimpleSubscriber
{
public:
  SimpleSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    std::string & topic_name,
    rclcpp::QoS & qos)
  : SimpleSubscriber(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_logging_interface(),
      topic_name,
      qos)
  {}

  SimpleSubscriber(
    rclcpp::Node::SharedPtr node,
    std::string & topic_name,
    rclcpp::QoS & qos)
  : SimpleSubscriber(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_logging_interface(),
      topic_name,
      qos)
  {}

  SimpleSubscriber(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    std::string & topic_name,
    rclcpp::QoS & qos)
  : node_base_(node_base),
    node_topics_(node_topics),
    node_logging_(node_logging),
    topic_name_(topic_name)
  {
    subscriber_ = rclcpp::create_subscription<SubscriberT>(node_topics_, topic_name_,
        qos, std::bind(&SimpleSubscriber::subscriberCallback, this, std::placeholders::_1));
  }

  std::shared_ptr<SubscriberT> getMsg()
  {
    if (!received_) {
      std::string error(topic_name_ + " not yet received");
      throw std::runtime_error(error);
    }
    return msg_;
  }

  ~SimpleSubscriber() {}

protected:
  void subscriberCallback(const std::shared_ptr<SubscriberT> msg)
  {
    msg_ = msg;
    if (!received_) {
      received_ = true;
    }
  }

  // Interfaces used for logging and creating publishers and subscribers
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  std::string topic_name_;
  std::shared_ptr<SubscriberT> msg_;
  bool received_{false};
  typename rclcpp::Subscription<SubscriberT>::SharedPtr subscriber_;
};

}  // namespace nav2_util

#endif  // NAV2_CLIENT_UTIL__SIMPLE_SUBSCRIBER_HPP_
