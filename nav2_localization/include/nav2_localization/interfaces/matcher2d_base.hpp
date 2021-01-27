// Copyright (c) 2021 Jose M. TORRES-CAMARA and Khaled SAAD

#ifndef NAV2_LOCALIZATION__INTERFACES__MATCHER2D_BASE_HPP_
#define NAV2_LOCALIZATION__INTERFACES__MATCHER2D_BASE_HPP_

#include <memory>  // For shared_ptr<>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_localization
{

/**
 * @class Matcher2d
 * @brief Abstract interface for a 2D matcher for localization purposes to adhere to with pluginlib
 */

class Matcher2d
{
public:
    Matcher2d(){}

  using Ptr = std::shared_ptr<nav2_localization::Matcher2d>;

  /**
   * @brief Returns the probability of the robot being at the provided pose, given the provided scan
   * @param scan A sensor scan
   * @param curr_pose A pose to match the scan from.
   * @return The probability of the robot being at curr_pose, given scan
   */
  virtual double getScanProbability(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan,
    const geometry_msgs::msg::TransformStamped &curr_pose) = 0;

  /**
   * @brief Sets the map of the environment, against which the matcher will compare sensor readings.
   * @param map A map of the environment
   */
  virtual void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map) = 0;

  /**
   * @brief Sets the pose of the sensor relative to the center of the robot's base.
   * @param sensor_pose The pose of the sensor relative to the center of the robot's base
   */
  virtual void setSensorPose(const geometry_msgs::msg::TransformStamped &sensor_pose) = 0;

  /**
     * @brief Configures the model, during the "Configuring" state of the parent lifecycle node.
     * @param node Pointer to the parent lifecycle node.
     */
  virtual void configure(const nav2_util::LifecycleNode::SharedPtr &node) = 0;

  /**
     * @brief Activates the model, during the "Activating" state of the parent lifecycle node.
     */
  virtual void activate() = 0;

  /**
     * @brief Deactivates the model, during the "Deactivating" state of the parent lifecycle node. 
     */
  virtual void deactivate() = 0;

  /**
     * @brief Cleans up the model, during the "Cleaningup" state of the parent lifecycle node.
     */
  virtual void cleanup() = 0;


protected:
  nav2_util::LifecycleNode::SharedPtr node_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;  // Reference to the 2D occupancy grid map of the environment where the robot is
  geometry_msgs::msg::TransformStamped sensor_pose_;  // The pose of the sensor relative to the center of the robot's base
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__INTERFACES__MATCHER2D_BASE_HPP_
