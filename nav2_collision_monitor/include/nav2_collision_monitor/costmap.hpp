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

#ifndef NAV2_COLLISION_MONITOR__COSTMAP_HPP_
#define NAV2_COLLISION_MONITOR__COSTMAP_HPP_

/**
 * @file costmap.hpp
 * @brief Observation source that converts a Nav2 costmap topic into 2D points for Collision Monitor.
 */

#include <memory>
#include <string>
#include <vector>

#include "nav2_collision_monitor/source.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_ros_common/subscription.hpp>

namespace nav2_collision_monitor
{

/**
 * @class CostmapSource
 * @brief Reads nav2_msgs::msg::Costmap and produces obstacle points for Collision Monitor.
 *
 * Cells with cost >= @ref cost_threshold_ are exported as points. Optionally, NO_INFORMATION (255)
 * can be treated as obstacles via @ref treat_unknown_as_obstacle_.
 *
 * Parameters (declared/queried in @ref getParameters):
 * - `topic` (std::string): costmap topic name (relative is recommended).
 * - `cost_threshold` (int, 0..255): minimum cost to consider a cell occupied.
 * - `treat_unknown_as_obstacle` (bool): whether cost 255 should be treated as occupied.
 */
class CostmapSource : public Source
{
public:
  /**
   * @brief Construct a CostmapSource.
   * @param node Weak pointer to the lifecycle node.
   * @param source_name Logical name of this source instance (for params/logs).
   * @param tf_buffer Shared TF buffer for frame transforms.
   * @param base_frame_id Robot base frame (e.g., "base_footprint").
   * @param global_frame_id Global frame of the costmap (e.g., "odom" or "map").
   * @param transform_tolerance Allowed TF age for transforms.
   * @param source_timeout Max age of data before it is considered stale.
   * @param base_shift_correction Whether to compensate robot motion during simulation checks.
   */
  CostmapSource(
    const nav2::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);

  /// @brief Destructor.
  ~CostmapSource();

  /**
   * @brief Declare and get parameters; create the subscription.
   *
   * Must be called during the node’s configuration phase (after construction, before use).
   * Reads `topic`, `cost_threshold`, and `treat_unknown_as_obstacle`.
   */
  void configure();

  /**
   * @brief Produce current obstacle points from the latest costmap.
   * @param curr_time Current time used for staleness checks and TF queries.
   * @param[out] data Output vector of points in the base frame.
   * @return true if valid, non-stale data were produced; false otherwise.
   *
   * @details
   * - Returns false if no message has arrived or data are older than @ref source_timeout_.
   * - Transforms points from costmap frame to @ref base_frame_id using @ref tf_buffer_.
   * - Applies @ref cost_threshold_ and @ref treat_unknown_as_obstacle_.
   */
  bool getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) override;

  /**
   * @brief Read parameters specific to the costmap source.
   * @param[out] source_topic Resolved topic name to subscribe to.
   *
   * Declares/gets: `topic`, `cost_threshold`, `treat_unknown_as_obstacle`.
   */
  void getParameters(std::string & source_topic);

  /**
   * @brief Check if costmap data has been received.
   * @return true if data has been received, false otherwise.
   */
  bool hasData() const {return data_ != nullptr;}

private:
  /**
   * @brief Subscription callback to store the latest costmap message.
   * @param msg Incoming costmap.
   */
  void dataCallback(nav2_msgs::msg::Costmap::ConstSharedPtr msg);

  /// @brief Latest costmap message.
  nav2_msgs::msg::Costmap::ConstSharedPtr data_;

  /// @brief Subscription handle for the costmap topic.
  nav2::Subscription<nav2_msgs::msg::Costmap>::SharedPtr data_sub_;

  /**
   * @brief Minimum cost (0..255) considered as an obstacle.
   * @note Typical values: 253 (inscribed), 254 (lethal). Inflation = 1..252.
   */
  int cost_threshold_{253};

  /**
   * @brief Whether cost 255 (NO_INFORMATION) is treated as an obstacle.
   */
  bool treat_unknown_as_obstacle_{true};
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COSTMAP_HPP_
