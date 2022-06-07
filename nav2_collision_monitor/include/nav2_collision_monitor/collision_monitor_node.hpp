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
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/time.h"
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

/**
 * @brief Collision Monitor ROS2 node
 */
class CollisionMonitor : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_collision_safery::CollisionMonitor.
   * Sets class variables, declares ROS-parameters
   */
  CollisionMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_collision_safery::CollisionMonitor.
   * Deletes allocated resources
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
   * @brief: Deactivates LifecyclePublishers, polygons and main processor
   * resets opeartion states to their defaults, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all subscribers/publishers, polygons/data sources arrays and velocity.
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
   * @brief Callback for odometry. Sets velocity to a given value.
   * @param msg Incoming odom message
   */
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  /**
   * @brief Tells whether a valid velocity was received
   * @param curr_time Current node time for odometry verification
   * @return Velocity is valid or not
   */
  bool velocityValid(const rclcpp::Time & curr_time);
  /**
   * @brief Callback for input cmd_vel
   * @param msg Input cmd_vel message
   */
  void cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /**
   * @brief Publishes output cmd_vel
   * @param vel Output velocity to publish
   */
  void publishVelocity(const Velocity & vel);

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param odom_topic Output name of odom topic
   * @param cmd_vel_in_topic Output name of cmd_vel_in topic
   * @param cmd_vel_out_topic Output name of cmd_vel_out topic
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & odom_topic,
    std::string & cmd_vel_in_topic,
    std::string & cmd_vel_out_topic);

  /**
   * @brief Main processing routine
   */
  void processMain();

  /**
   * @brief Processes the polygon of STOP and SLOWDOWN action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   */
  void processStopSlowdown(
    const std::shared_ptr<PolygonBase> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    Action & robot_action);
  /**
   * @brief Processes the polygon of APPROACH action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Current robot velocity
   * @param min_collision_time Minimum time to collision
   * @param robot_action Output processed robot action
   */
  void processApproach(
    const std::shared_ptr<PolygonBase> polygon,
    const std::vector<Point> & collision_points,
    const Velocity & velocity,
    double & min_collision_time,
    Action & robot_action);

  /**
   * @brief Accelerates robot from APPROACH to its normal operation:
   * this gradual speed increase is a smoothing, made to wipe out
   * twitching of the robot conditioned by sensors data noise.
   * @param curr_time Current node time for velocity change calculation
   * @param desired_vel Maximum desired robot velocity
   * @param robot_action Output processed robot action
   */
  void releaseOperation(
     const rclcpp::Time & curr_time, const Velocity & desired_vel, Action & robot_action);

  /**
   * @brief Prints robot action and its speed, in case if robot is not in normal operation
   * @param robot_action Robot action to print
   */
  void printAction(const Action & robot_action);

  /**
   * @brief Polygons publishing routine. Made for better visualization.
   */
  void publishPolygons();

  // ----- Variables -----

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief Polygons array
  std::vector<std::shared_ptr<PolygonBase>> polygons_;

  /// @brief Data sources array
  std::vector<std::shared_ptr<SourceBase>> sources_;

  /// @brief Whether very first odom message received
  bool first_odom_;
  /// @brief Odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Working with odom velocity
  /// @brief Latest velocity
  Velocity velocity_;
  /// @brief Whether the velocity was received
  bool velocity_received_;
  /// @brief Latest velocity timestamp
  rclcpp::Time velocity_stamp_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_in_sub_;
  /// @brief Latest input cmd_vel
  Velocity cmd_vel_in_;
  /// @brief Output cmd_vel publisher
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;

  /// @brief Maximum time interval when data is considered valid
  tf2::Duration data_timeout_;

  /// @brief Whether main rotuine is active
  bool process_active_;

  /// @brief Previous robot action state
  Action robot_action_prev_;
  /// @brief Whether gradually release robot to normal operation
  bool release_operation_;
  /// @brief Robot acceleration when increase the speed towards to normal operation
  double release_acceleration_;
  /// @brief Previous timestamp. Used for robot acceleration.
  rclcpp::Time prev_time_;
};  // class CollisionMonitor

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
