// Copyright (c) 2024 Tony Najjar
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

#ifndef NAV2_LOOPBACK_SIM__LOOPBACK_SIMULATOR_HPP_
#define NAV2_LOOPBACK_SIM__LOOPBACK_SIMULATOR_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/twist_subscriber.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

namespace nav2_loopback_sim
{

/**
 * @brief A loopback simulator that replaces a physics simulator to create a
 * frictionless, inertialess, and collisionless simulation environment. It
 * accepts cmd_vel messages and publishes odometry & TF messages based on the
 * cumulative velocities received to mimic global localization and simulation.
 * It also accepts initialpose messages to set the initial pose of the robot.
 */
class LoopbackSimulator : public nav2::LifecycleNode
{
public:
  explicit LoopbackSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LoopbackSimulator() = default;

protected:
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);
  void cmdVelStampedCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void setupTimerCallback();
  void timerCallback();
  void odomTimerCallback();
  void publishLaserScan();

  // Helpers
  void getBaseToLaserTf();
  void getMap();
  void publishTransforms(
    geometry_msgs::msg::TransformStamped & map_to_odom,
    geometry_msgs::msg::TransformStamped & odom_to_base_link);
  void publishOdometry(const geometry_msgs::msg::TransformStamped & odom_to_base_link);
  std::tuple<double, double, double> getLaserPose();
  void getLaserScan(int num_samples, sensor_msgs::msg::LaserScan & scan_msg);

  static geometry_msgs::msg::Quaternion addYawToQuat(
    const geometry_msgs::msg::Quaternion & quaternion, double yaw_to_add);

  // Parameters
  double update_dur_;
  std::string base_frame_id_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string scan_frame_id_;
  double odom_publish_dur_;
  double scan_publish_dur_;
  bool publish_map_odom_tf_;
  double scan_range_min_;
  double scan_range_max_;
  double scan_angle_min_;
  double scan_angle_max_;
  double scan_angle_increment_;
  bool use_inf_;
  bool publish_scan_;

  // State
  std::optional<geometry_msgs::msg::Twist> curr_cmd_vel_;
  rclcpp::Time curr_cmd_vel_time_;
  bool has_initial_pose_{false};
  geometry_msgs::msg::Pose initial_pose_;
  bool has_map_{false};
  nav_msgs::msg::OccupancyGrid map_;
  bool has_base_to_laser_{false};
  tf2::Transform tf_base_to_laser_;

  // Transforms
  geometry_msgs::msg::TransformStamped t_map_to_odom_;
  geometry_msgs::msg::TransformStamped t_odom_to_base_link_;

  // ROS interfaces
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav2::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  std::unique_ptr<nav2_util::TwistSubscriber> cmd_vel_sub_;

  nav2::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav2::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

  rclcpp::TimerBase::SharedPtr setup_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

}  // namespace nav2_loopback_sim

#endif  // NAV2_LOOPBACK_SIM__LOOPBACK_SIMULATOR_HPP_
