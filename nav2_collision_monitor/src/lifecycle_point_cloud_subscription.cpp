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

namespace nav2_collision_monitor
{

LifecyclePointCloudSubscription::LifecyclePointCloudSubscription(
  rclcpp_lifecycle::LifecycleNode & node)
{
  pct_ = std::make_shared<point_cloud_transport::PointCloudTransport>(node);
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
  auto gated = [this, cb = std::move(callback)](
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      if (is_activated()) {
        cb(msg);
      }
    };
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  data_sub_ = pct_->subscribe(topic, qos, gated, {}, &transport_hints);
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
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
