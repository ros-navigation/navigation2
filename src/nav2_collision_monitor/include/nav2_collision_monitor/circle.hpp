// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COLLISION_MONITOR__CIRCLE_HPP_
#define NAV2_COLLISION_MONITOR__CIRCLE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "std_msgs/msg/float32.hpp"

#include "nav2_collision_monitor/polygon.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Circle shape implementaiton.
 * For STOP/SLOWDOWN/LIMIT model it represents zone around the robot
 * while for APPROACH model it represents robot footprint.
 */
class Circle : public Polygon
{
public:
  /**
   * @brief Circle class constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of circle
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   */
  Circle(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);
  /**
   * @brief Circle class destructor
   */
  ~Circle();

  /**
   * @brief Gets polygon points, approximated to the circle.
   * To be used in visualization purposes.
   * @param poly Output polygon points (vertices)
   */
  void getPolygon(std::vector<Point> & poly) const override;

  /**
   * @brief Gets number of points inside circle
   * @param points Input array of points to be checked
   * @return Number of points inside circle. If there are no points,
   * returns zero value.
   */
  int getPointsInside(const std::vector<Point> & points) const override;

  /**
   * @brief Returns true if circle radius is set.
   * Otherwise, prints a warning and returns false.
   */
  bool isShapeSet() override;

protected:
  /**
   * @brief Supporting routine obtaining polygon-specific ROS-parameters
   * @param polygon_sub_topic Input name of polygon subscription topic
   * @param polygon_pub_topic Output name of polygon or radius publishing topic
   * @param footprint_topic Output name of footprint topic. For Circle returns empty string,
   * there is no footprint subscription in this class.
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(
    std::string & polygon_sub_topic,
    std::string & polygon_pub_topic,
    std::string & footprint_topic) override;

  /**
   * @brief Creates polygon or radius topic subscription
   * @param polygon_sub_topic Output name of polygon or radius subscription topic.
   * Empty, if no polygon subscription.
   */
  void createSubscription(std::string & polygon_sub_topic) override;

  /**
   * @brief Updates polygon from radius value
   * @param radius New circle radius to update polygon
   */
  void updatePolygon(double radius);

  /**
   * @brief Dynamic circle radius callback
   * @param msg Shared pointer to the radius value message
   */
  void radiusCallback(std_msgs::msg::Float32::ConstSharedPtr msg);


  // ----- Variables -----

  /// @brief Radius of the circle
  double radius_;
  /// @brief (radius * radius) value. Stored for optimization.
  double radius_squared_ = -1.0;
  /// @brief Radius subscription
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr radius_sub_;
};  // class Circle

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__CIRCLE_HPP_
