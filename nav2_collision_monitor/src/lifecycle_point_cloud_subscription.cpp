// Copyright (c) 2025 Angsa Robotics
// Copyright (c) 2025 lotusymt
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

#include "nav2_collision_monitor/lifecycle_point_cloud_subscription.hpp"

#include "rclcpp/node.hpp"
#if defined(POINT_CLOUD_TRANSPORT_SUBSCRIBE_USE_QOS)
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#endif

namespace nav2_collision_monitor
{

LifecyclePointCloudSubscription::LifecyclePointCloudSubscription(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: node_(node)
{
#if defined(POINT_CLOUD_TRANSPORT_SUBSCRIBE_USE_QOS)
  // Rolling: LifecycleNode does not inherit Node; use node interfaces
  pct_ = std::make_shared<point_cloud_transport::PointCloudTransport>(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>(
      node->get_node_base_interface(),
      node->get_node_parameters_interface(),
      node->get_node_topics_interface(),
      node->get_node_logging_interface()));
#else
  // Jazzy/Kilted: PointCloudTransport(rclcpp::Node::SharedPtr)
  rclcpp::Node * node_ptr = dynamic_cast<rclcpp::Node *>(node.get());
  if (!node_ptr) {
    throw std::invalid_argument(
      "LifecyclePointCloudSubscription: node must be a rclcpp::Node "
      "(LifecycleNode inheriting Node)");
  }
  pct_ = std::make_shared<point_cloud_transport::PointCloudTransport>(
    std::shared_ptr<rclcpp::Node>(node, node_ptr));
#endif
}

LifecyclePointCloudSubscription::~LifecyclePointCloudSubscription()
{
  shutdown();
}

void LifecyclePointCloudSubscription::subscribe(
  const std::string & topic,
  const rclcpp::QoS & qos,
  Callback callback,
  const point_cloud_transport::TransportHints & transport_hints)
{
  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> gated =
    [this, cb = std::move(callback)](
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      if (is_activated()) {
        cb(msg);
      }
    };
#if defined(POINT_CLOUD_TRANSPORT_SUBSCRIBE_USE_QOS)
  data_sub_ = pct_->subscribe(topic, qos, gated, std::shared_ptr<void>(), &transport_hints);
#else
  data_sub_ = pct_->subscribe(
    topic,
    qos.get_rmw_qos_profile(),
    gated,
    std::shared_ptr<void>(),
    &transport_hints);
#endif
}

void LifecyclePointCloudSubscription::on_activate()
{
  rclcpp_lifecycle::SimpleManagedEntity::on_activate();
}

void LifecyclePointCloudSubscription::on_deactivate()
{
  rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
}

void LifecyclePointCloudSubscription::shutdown()
{
  data_sub_.shutdown();
}

}  // namespace nav2_collision_monitor
