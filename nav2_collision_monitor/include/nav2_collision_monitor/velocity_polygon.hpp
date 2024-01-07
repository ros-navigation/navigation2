// Copyright (c) 2023 Dexory
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

#ifndef NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/types.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_collision_monitor
{

/**
 * @brief Velocity polygon class.
 * This class contains all the points of the polygon and
 * the expected condition of the velocity based polygon.
 */
class VelocityPolygon : public Polygon
{
public:
  /**
   * @brief VelocityPolygon constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of main polygon
   */
  VelocityPolygon(
    const nav2_util::LifecycleNode::WeakPtr & node, const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  /**
   * @brief VelocityPolygon destructor
   */
  virtual ~VelocityPolygon();

  /**
   * @brief Supporting routine obtaining velocity polygon specific ROS-parameters
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & /*polygon_sub_topic*/, std::string & polygon_pub_topic,
    std::string & /*footprint_topic*/) override;

protected:
  // override the base class update polygon
  void updatePolygon(const Velocity & cmd_vel_in) override;

  bool holonomic_;

  // Define a structure to store the basic parameters
  struct SubPolygonParameter
  {
    std::vector<Point> poly_;
    std::string velocity_polygon_name_;
    double linear_min_;
    double linear_max_;
    double theta_min_;
    double theta_max_;
    double direction_end_angle_;
    double direction_start_angle_;
  };

  // Create a vector to store instances of BasicParameters
  std::vector<SubPolygonParameter> sub_polygons_;

  /**
   * @brief Check if the velocities and direction is in expected range.
   * @param cmd_vel_in Robot twist command input
   * @return True if speed and direction is within the condition
   */
  bool isInRange(const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon_param);

};  // class VelocityPolygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
