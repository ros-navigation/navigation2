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

#ifndef NAV2_COLLISION_MONITOR__LIFECYCLE_POINT_CLOUD_SUBSCRIPTION_HPP_
#define NAV2_COLLISION_MONITOR__LIFECYCLE_POINT_CLOUD_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "point_cloud_transport/point_cloud_transport.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Lifecycle-managed subscription that wraps point_cloud_transport::Subscriber.
 *
 * Gates message delivery on activation state (on_activate/on_deactivate), so it can be
 * used like nav2::Subscription when using point_cloud_transport for transport hints.
 */
class LifecyclePointCloudSubscription : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecyclePointCloudSubscription)

  using Callback = std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)>;

  /**
   * @brief Construct and create the PointCloudTransport.
   * @param node Lifecycle node shared pointer (PointCloudTransport requires Node::SharedPtr on ROS 2 Jazzy).
   */
  explicit LifecyclePointCloudSubscription(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  ~LifecyclePointCloudSubscription();

  /**
   * @brief Subscribe to a topic with transport hints; callback is only invoked when activated.
   * @param topic Topic name
   * @param qos QoS profile
   * @param callback User callback (invoked only when this entity is activated)
   * @param transport_hints Transport hints for point_cloud_transport
   */
  void subscribe(
    const std::string & topic,
    const rclcpp::QoS & qos,
    Callback callback,
    const point_cloud_transport::TransportHints & transport_hints);

  void on_activate() override;
  void on_deactivate() override;

  /** @brief Unsubscribe and release the underlying subscriber. */
  void shutdown();

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<point_cloud_transport::PointCloudTransport> pct_;
  point_cloud_transport::Subscriber data_sub_;
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__LIFECYCLE_POINT_CLOUD_SUBSCRIPTION_HPP_
