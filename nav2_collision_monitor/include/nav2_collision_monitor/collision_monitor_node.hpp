// Copyright (c) 2022 Samsung Research Russia
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

#ifndef NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon_base.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/circle.hpp"
#include "nav2_collision_monitor/source_base.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"

namespace nav2_collision_monitor
{

class CollisionMonitorNode : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_collision_safery::CollisionMonitorNode.
   * Sets class variables, declares ROS-parameters
   */
  CollisionMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_collision_safery::CollisionMonitorNode.
   * Deletes allocated resources
   */
  ~CollisionMonitorNode();

protected:
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates LifecyclePublishers and main worker, creates bond connection,
   * creates polygon publisher thread
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates LifecyclePublishers and main worker,
   * resets opeartion states to their defaults, stops polygon publisher thread,
   * destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers/threads timers,
   * deletes allocated resources, resets velocity
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  // @brief Polygons publishing routine (to be used as callback).
  // Made for better visualization.
  void publishPolygons();

  // @brief Callback for odometry. Sets velocity to a given value.
  // @param msg Incoming odom message
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // @brief Get latest detected velocity
  // @return Latest velocity
  Velocity getVelocity();
  // @brief Tells whether a valid velocity was received
  // @return Velocity is valid or not
  bool velocityValid();
  // @brief Resets velocity to its initial non-valid state
  void resetVelocity();

  // @brief Callback for input cmd_vel
  // @param msg Input cmd_vel message
  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  // @brief Gets input cmv_vel
  // @return Input cmd_vel
  Velocity getCmdVelIn();

  // @brief Publishes output cmd_vel
  // @param vel Output velocity to publish
  void publishVelocity(const Velocity & vel);

  // @brief Supporting routine obtaining all ROS-parameters
  // @return True if all parameters were obtained or false in failure case
  bool getParameters();

  // @brief Worker main routine
  void workerMain();
  // @brief Sets main worker to a given state
  // @param worker_active State for worker (active or non-active)
  void setWorkerActive(const bool worker_active);
  // @brief Gets main worker state
  // @return Curret state of main worker routine
  bool getWorkerActive();

  // ----- Variables -----

  // Transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Polygons
  std::vector<std::string> polygon_names_;
  std::vector<std::shared_ptr<PolygonBase>> polygons_;
  std::string polygon_topic_;
  geometry_msgs::msg::Polygon polygons_pub_;  // All polygons points stored for publisging
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::TimerBase::SharedPtr polygon_pub_timer_;

  // Data sources
  std::vector<std::string> source_names_;
  std::vector<std::shared_ptr<SourceBase>> sources_;

  // Odom callback
  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Working with velocity
  Velocity velocity_;
  bool velocity_valid_;
  mutex_t velocity_mutex_;

  // Input/output speed controls
  std::string cmd_vel_in_topic_, cmd_vel_out_topic_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  Velocity cmd_vel_in_;
  mutex_t cmd_vel_in_mutex_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;

  // Global parameters
  std::string base_frame_id_;
  double max_time_shift_;

  // Working with main routine
  bool worker_active_;
  mutex_t worker_active_mutex_;

  // Previous robot action state
  Action ra_prev_;
  // Gradually release normal robot operation
  bool release_operation_;
  // Step to increase robot speed towards to normal operation
  double release_step_;
};  // class CollisionMonitorNode

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
