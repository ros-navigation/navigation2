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
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/circle.hpp"
#include "nav2_collision_monitor/source.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Collision Monitor ROS2 node
 */
class CollisionMonitor : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_collision_safery::CollisionMonitor
   * @param options Additional options to control creation of the node.
   */
  CollisionMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_collision_safery::CollisionMonitor
   */
  ~CollisionMonitor();

protected:
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main subscribers and publishers,
   * creates polygons and data sources objects
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates LifecyclePublishers, polygons and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates LifecyclePublishers, polygons and main processor, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers, polygons/data sources arrays
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Callback for odometry. Sets odom_frame_id_ from message.
   * @param msg Incoming odom message
   */
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  /**
   * @brief Callback for input cmd_vel
   * @param msg Input cmd_vel message
   */
  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /**
   * @brief Publishes output cmd_vel
   * @param vel Output velocity to publish
   */
  void publishVelocity(const Velocity & vel) const;

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param odom_topic Output name of odom topic
   * @param cmd_vel_in_topic Output name of cmd_vel_in topic
   * @param cmd_vel_out_topic Output name of cmd_vel_out topic
   * @param footprint_topic Output name of footprint topic. Empty, if no footprint subscription
   * is required.
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & odom_topic,
    std::string & cmd_vel_in_topic,
    std::string & cmd_vel_out_topic,
    std::string & footprint_topic);
  /**
   * @brief Supporting routine creating and configuring all polygons
   * @return True if all polygons were configured successfully or false in failure case
   */
  bool configurePolygons();
  /**
   * @brief Supporting routine creating and configuring robot footprint polygon
   * @param footprint_topic Output name of footprint topic. Empty, if no footprint subscription
   * is required.
   * @return True if footprint polygon was configured successfully or false in failure case
   */
  bool configureFootprint(std::string & footprint_topic);
  /**
   * @brief Supporting routine creating and configuring all data sources
   * @return True if all sources were configured successfully or false in failure case
   */
  bool configureSources();

  /**
   * @brief Main processing routine
   * @param cmd_vel_in Input desired robot velocity
   */
  void process(const Velocity & cmd_vel_in);

  /**
   * @brief Processes the polygon of STOP and SLOWDOWN action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise false
   */
  bool processStopSlowdown(
    const std::shared_ptr<Polygon> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  /**
   * @brief Processes APPROACH action type
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by APPROACH model of robot footprint, otherwise false
   */
  bool processApproach(
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action) const;

  /**
   * @brief Obtains estimated (simulated) time before a collision.
   * Applicable for APPROACH model.
   * @param collision_points Array of 2D obstacle points
   * @param velocity Simulated robot velocity
   * @return Estimated time before a collision. If there is no collision,
   * return value will be negative.
   */
  double getCollisionTime(
    const std::vector<Point> & collision_points,
    const Velocity & velocity) const;

  /**
   * @brief Prints robot action and polygon caused it (if it was)
   * @param robot_action Robot action to print
   * @param action_polygon Pointer to a polygon causing a selected action
   */
  void printAction(
    const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const;

  /**
   * @brief Polygons publishing routine. Made for visualization.
   */
  void publishPolygons() const;

  // ----- Variables -----

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief STOP/SLOWDOWN polygons array
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief Footprint APPROACH polygon
  std::shared_ptr<Polygon> footprint_;

  /// @brief Data sources array
  std::vector<std::shared_ptr<Source>> sources_;

  /// @brief Odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  /// @brief Output cmd_vel publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;

  // Global parameters
  /// @brief Base frame ID
  std::string base_frame_id_;
  /// @brief Odometry frame ID. Used as global frame to get source->base time inerpolated transform.
  std::string odom_frame_id_;
  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;
  /// @brief Maximum time interval in which data is considered valid
  tf2::Duration data_timeout_;
  /// @brief Whether to use APPROACH model
  bool approach_;
  /// @brief Footprint subscriber
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  /// @brief Time before collision in seconds
  double time_before_collision_;
  /// @brief Time step for robot movement simulation
  double simulation_time_step_;

  /// @brief Whether main routine is active
  bool process_active_;

  /// @brief Previous robot action type
  ActionType action_type_prev_;
};  // class CollisionMonitor

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
