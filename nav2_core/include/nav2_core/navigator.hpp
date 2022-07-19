// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_CORE__NAVIGATOR_HPP_
#define NAV2_CORE__NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_core
{

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms and reference frames.
 */
struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/**
 * @class NavigatorMuxer
 * @brief A class to control the state of the BT navigator by allowing only a single
 * plugin to be processed at a time.
 */
class NavigatorMuxer
{
public:
  /**
   * @brief A Navigator Muxer constructor
   */
  NavigatorMuxer()
  : current_navigator_(std::string("")) {}

  /**
   * @brief Get the navigator muxer state
   * @return bool If a navigator is in progress
   */
  bool isNavigating()
  {
    std::scoped_lock l(mutex_);
    return !current_navigator_.empty();
  }

  /**
   * @brief Start navigating with a given navigator
   * @param string Name of the navigator to start
   */
  void startNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation requested while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    }
    current_navigator_ = navigator_name;
  }

  /**
   * @brief Stop navigating with a given navigator
   * @param string Name of the navigator ending task
   */
  void stopNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation stopped while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    } else {
      current_navigator_ = std::string("");
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};

class Navigator
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Navigator)

  /**
   * @brief Virtual destructor
   */
  virtual ~Navigator() {}

  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> &,
    const FeedbackUtils & feedback_utils,
    nav2_core::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) = 0;

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return true If success
   */
  virtual bool activate() = 0;

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return true If success
   */
  virtual bool deactivate() = 0;

  /**
   * @brief Cleanup a navigator
   * @return true If success
   */
  virtual bool cleanup() = 0;

  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__NAVIGATOR_HPP_
