/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__COSTMAP_2D_ROS_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_2D_ROS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon.h"
#include "geometry_msgs/msg/polygon_stamped.h"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "tf2/transform_datatypes.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief  Constructor for the wrapper, the node will
   * be placed in a namespace equal to the node's name
   * @param name Name of the costmap ROS node
   */
  explicit Costmap2DROS(const std::string & name);

  /**
   * @brief  Constructor for the wrapper
   * @param name Name of the costmap ROS node
   * @param parent_namespace Absolute namespace of the node hosting the costmap node
   * @param local_namespace Namespace to append to the parent namespace
   */
  explicit Costmap2DROS(
    const std::string & name,
    const std::string & parent_namespace,
    const std::string & local_namespace);

  ~Costmap2DROS();

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause();

  /**
   * @brief  Resumes costmap updates
   */
  void resume();

  void updateMap();

  /**
   * @brief Reset each individual layer
   */
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent()
  {
    return layered_costmap_->isCurrent();
  }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);

  /** @brief Returns costmap name */
  std::string getName() const
  {
    return name_;
  }

  /** @brief Returns the delay in transform (tf) data that is tolerable in seconds */
  double getTransformTolerance() const
  {
    return transform_tolerance_;
  }

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap().
   */
  Costmap2D * getCostmap()
  {
    return layered_costmap_->getCostmap();
  }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  std::string getGlobalFrameID()
  {
    return global_frame_;
  }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  std::string getBaseFrameID()
  {
    return robot_base_frame_;
  }

  LayeredCostmap * getLayeredCostmap()
  {
    return layered_costmap_;
  }

  /** @brief Returns the current padded footprint as a geometry_msgs::msg::Polygon. */
  geometry_msgs::msg::Polygon getRobotFootprintPolygon()
  {
    return nav2_costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::msg::Point> getRobotFootprint()
  {
    return padded_footprint_;
  }

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::msg::Point> getUnpaddedRobotFootprint()
  {
    return unpadded_footprint_;
  }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint);

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setRobotFootprintPolygon(const geometry_msgs::msg::Polygon::SharedPtr footprint);

  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() {return tf_buffer_;}

  /**
   * @brief  Get the costmap's use_radius_ parameter, corresponding to
   * whether the footprint for the robot is a circle with radius robot_radius_
   * or an arbitrarily defined footprint in footprint_.
   * @return  use_radius_
   */
  bool getUseRadius() {return use_radius_;}

protected:
  rclcpp::Node::SharedPtr client_node_;

  // Publishers and subscribers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
    footprint_pub_;
  Costmap2DPublisher * costmap_publisher_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_;

  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  LayeredCostmap * layered_costmap_{nullptr};
  std::string name_;
  std::string parent_namespace_;
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_{false};
  bool stop_updates_{false};
  bool initialized_{false};
  bool stopped_{true};
  std::thread * map_update_thread_{nullptr};  ///< @brief A thread for updating the map
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_cycle_{1, 0};
  pluginlib::ClassLoader<Layer> plugin_loader_{"nav2_costmap_2d", "nav2_costmap_2d::Layer"};

  // Parameters
  void getParameters();
  bool always_send_full_costmap_{false};
  std::string footprint_;
  float footprint_padding_{0};
  std::string global_frame_;       ///< The global frame for the costmap
  int map_height_meters_{0};
  double map_publish_frequency_{0};
  double map_update_frequency_{0};
  int map_width_meters_{0};
  double origin_x_{0};
  double origin_y_{0};
  std::vector<std::string> default_plugins_;
  std::vector<std::string> default_types_;
  std::vector<std::string> plugin_names_;
  std::vector<std::string> plugin_types_;
  double resolution_{0};
  std::string robot_base_frame_;   ///< The frame_id of the robot base
  double robot_radius_;
  bool rolling_window_{false};     ///< Whether to use a rolling window version of the costmap
  bool track_unknown_space_{false};
  double transform_tolerance_{0};  ///< The timeout before transform errors

  // Derived parameters
  bool use_radius_{false};
  std::vector<geometry_msgs::msg::Point> unpadded_footprint_;
  std::vector<geometry_msgs::msg::Point> padded_footprint_;

  std::unique_ptr<ClearCostmapService> clear_costmap_service_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_2D_ROS_HPP_
