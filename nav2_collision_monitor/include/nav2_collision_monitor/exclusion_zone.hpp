// Copyright (c) 2026 Dexory
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

#ifndef NAV2_COLLISION_MONITOR__EXCLUSION_ZONE_HPP_
#define NAV2_COLLISION_MONITOR__EXCLUSION_ZONE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"

#include "nav2_ros_common/lifecycle_node.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Region that removes (masks out) source points falling inside it.
 *
 * Unlike a Polygon, an ExclusionZone does not produce a robot Action. It is a
 * pre-filter applied to a Source's collision points before the action polygons
 * are evaluated. Its shape is defined in an arbitrary frame (frame_id) and is
 * transformed into the robot base frame every cycle, so the zone can track a
 * moving TF frame (e.g. a charging station the robot is docking onto).
 *
 * Fail-safe: if the zone is enabled but its transform cannot be obtained, no
 * points are removed (full collision protection is retained).
 */
class ExclusionZone
{
public:
  /**
   * @brief ExclusionZone constructor
   * @param node Collision Monitor node pointer
   * @param zone_name Name of the exclusion zone
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID (source points are expressed in this frame)
   * @param transform_tolerance Transform tolerance
   */
  ExclusionZone(
    const nav2::LifecycleNode::WeakPtr & node,
    const std::string & zone_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);

  /**
   * @brief ExclusionZone destructor
   */
  ~ExclusionZone();

  /**
   * @brief Reads ROS parameters and configures the zone.
   * @return True if configured correctly, false otherwise
   */
  bool configure();

  /**
   * @brief Activates the visualization publisher (if any)
   */
  void activate();

  /**
   * @brief Deactivates the visualization publisher (if any)
   */
  void deactivate();

  /**
   * @brief Removes from data all points that fall inside the (enabled) zone.
   * No-op when the zone is disabled. Fail-safe (removes nothing) when the zone
   * transform is unavailable.
   * @param curr_time Current node time, used for the zone-frame -> base-frame lookup
   * @param data Source points in base_frame_id_; excluded points are erased in place
   */
  void apply(const rclcpp::Time & curr_time, std::vector<Point> & data) const;

  /**
   * @brief Obtains the name of the zone
   * @return Zone name
   */
  std::string getName() const;

  /**
   * @brief Obtains the enabled state of the zone
   * @return Whether the zone is enabled
   */
  bool getEnabled() const;

  /**
   * @brief Publishes the zone footprint for visualization (if enabled)
   */
  void publish() const;

protected:
  /**
   * @brief Reads ROS parameters for the zone
   * @return True if all parameters were obtained, false otherwise
   */
  bool getParameters();

  /**
   * @brief Apply parameter updates after validation (dynamic reconfigure)
   * @param parameters List of parameters that have been updated
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Validate incoming parameter updates before applying them
   * @param parameters List of parameters being updated
   * @return Result indicating whether the update is accepted
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};
  /// @brief Node clock (for throttled logging and message stamps)
  rclcpp::Clock::SharedPtr node_clock_;
  /// @brief Dynamic parameters handlers
  mutable std::mutex mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;

  /// @brief Name of the zone
  std::string zone_name_;

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief Robot base frame ID
  std::string base_frame_id_;
  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;

  /// @brief Frame the zone shape is anchored to (tracked via TF). Defaults to base_frame_id_.
  std::string frame_id_;
  /// @brief Whether the zone shape is a circle (otherwise polygon)
  bool is_circle_{false};
  /// @brief Zone polygon vertices, expressed in frame_id_ (for polygon type)
  std::vector<Point> poly_;
  /// @brief Circle radius (for circle type)
  double radius_{0.0};
  /// @brief radius squared, cached
  double radius_squared_{0.0};
  /// @brief Lower bound of the height band (base-frame z) a point must be within to be excluded
  double min_height_;
  /// @brief Upper bound of the height band (base-frame z) a point must be within to be excluded
  double max_height_;
  /// @brief Whether the zone is currently active
  bool enabled_{false};

  /// @brief Whether to publish the zone footprint for visualization
  bool visualize_{false};
  /// @brief Zone footprint publisher
  nav2::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr zone_pub_;
};  // class ExclusionZone

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__EXCLUSION_ZONE_HPP_
