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

#ifndef NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_update.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_type_adapter.hpp"

namespace nav2_costmap_2d
{
/**
 * @class CostmapSubscriber
 * @brief Subscribes to the costmap via a ros topic
 */
class CostmapSubscriber
{
public:
  /**
   * @brief A constructor
   */
  CostmapSubscriber(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & topic_name);

  template<typename NodeT>
  CostmapSubscriber(
    const NodeT & parent,
    const std::string & topic_name,
    const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr)
  : topic_name_(topic_name)
  {
    logger_ = parent->get_logger();

    costmap_sub_ = nav2::interfaces::create_subscription<nav2_costmap_2d::Costmap2DStamped>(
      parent, topic_name_,
      [this](const nav2_costmap_2d::Costmap2DStamped & msg) {
        costmapCallback(msg);
      },
      nav2::qos::LatchedSubscriptionQoS(3), callback_group);

    costmap_update_sub_ = nav2::interfaces::create_subscription<nav2_msgs::msg::CostmapUpdate>(
      parent, topic_name_ + "_updates",
      [this](const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & msg) {
        costmapUpdateCallback(msg);
      },
      nav2::qos::LatchedSubscriptionQoS(), callback_group);
  }

  /**
   * @brief A destructor
   */
  ~CostmapSubscriber() {}

  /**
   * @brief Get current costmap
   */
  std::shared_ptr<Costmap2D> getCostmap();
  /**
   * @brief Callback for the costmap topic
   */
  void costmapCallback(const nav2_costmap_2d::Costmap2DStamped & msg);
  /**
   * @brief Callback for the costmap's update topic
   */
  void costmapUpdateCallback(const nav2_msgs::msg::CostmapUpdate::ConstSharedPtr & update_msg);

  std::string getFrameID() const
  {
    return frame_id_;
  }

protected:
  bool isCostmapReceived() {return costmap_ != nullptr;}

  nav2::Subscription<nav2_costmap_2d::Costmap2DStamped>::SharedPtr costmap_sub_;
  nav2::Subscription<nav2_msgs::msg::CostmapUpdate>::SharedPtr costmap_update_sub_;

  std::shared_ptr<Costmap2D> costmap_;
  std::string topic_name_;
  std::string frame_id_;
  std::mutex costmap_msg_mutex_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
