// Copyright (c) 2022 Samsung Research
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

#ifndef NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "ruckig/ruckig.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_velocity_smoother
{

/**
 * @class nav2_velocity_smoother::VelocitySmoother
 * @brief This class that smooths cmd_vel velocities for robot bases
 */
class VelocitySmoother : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_velocity_smoother::VelocitySmoother
   * @param options Additional options to control creation of the node.
   */
  explicit VelocitySmoother(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for nav2_velocity_smoother::VelocitySmoother
   */
  ~VelocitySmoother();

protected:
  /**
   * @brief Configures parameters and member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Network interfaces
  std::unique_ptr<nav2_util::OdomSmoother> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr smoothed_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Clock::SharedPtr clock_;
  geometry_msgs::msg::Twist last_cmd_;

  // Parameters
  double smoothing_frequency_;
  bool open_loop_;
  double max_velocity_;
  double min_velocity_;
  double max_accel_;
  double min_accel_;
  double deadband_velocity_;
  rclcpp::Duration velocity_timeout_{0.0};
}

}  // namespace nav2_velocity_smoother

#endif  // NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_
