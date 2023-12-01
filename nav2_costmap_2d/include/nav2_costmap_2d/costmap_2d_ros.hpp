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

#include <atomic>
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
   * @brief  Constructor for the wrapper
   * @param options Additional options to control creation of the node.
   */
  Costmap2DROS(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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

  /**
   * @brief A destructor
   */
  ~Costmap2DROS();

  /**
   * @brief Configure node
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate node
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate node
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup node
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief shutdown node
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief as a child-LifecycleNode :
   * Costmap2DROS may be launched by another Lifecycle Node as a composed module
   * If composed, its parents will handle the shutdown, which includes this module
   */
  void on_rcl_preshutdown() override
  {
    if (is_lifecycle_follower_) {
      // Transitioning handled by parent node
      return;
    }

    // Else, if this is an independent node, this node needs to handle itself.
    RCLCPP_INFO(
      get_logger(), "Running Nav2 LifecycleNode rcl preshutdown (%s)",
      this->get_name());

    runCleanups();

    destroyBond();
  }

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

  /**
   * @brief Update the map with the layered costmap / plugins
   */
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

  /**
   * @brief Transform the input_pose in the global frame of the costmap
   * @param input_pose pose to be transformed
   * @param transformed_pose pose transformed
   * @return True if the pose was transformed successfully, false otherwise
   */
  bool transformPoseToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & transformed_pose);

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

  /**
   * @brief Get the layered costmap object used in the node
   */
  LayeredCostmap * getLayeredCostmap()
  {
    return layered_costmap_.get();
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

  /**
   * @brief  Get the costmap's robot_radius_ parameter, corresponding to
   * raidus of the robot footprint when it is defined as as circle
   * (i.e. when use_radius_ == true).
   * @return  robot_radius_
   */
  double getRobotRadius() {return robot_radius_;}

protected:
  // Publishers and subscribers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
    footprint_pub_;
  std::unique_ptr<Costmap2DPublisher> costmap_publisher_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_;

  // Dedicated callback group and executor for tf timer_interface and message fillter
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;

  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<LayeredCostmap> layered_costmap_{nullptr};
  std::string name_;
  std::string parent_namespace_;

  /**
   * @brief Function on timer for costmap update
   */
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_{false};
  std::atomic<bool> stop_updates_{false};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> stopped_{true};
  std::mutex _dynamic_parameter_mutex;
  std::unique_ptr<std::thread> map_update_thread_;  ///< @brief A thread for updating the map
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_cycle_{1, 0};
  pluginlib::ClassLoader<Layer> plugin_loader_{"nav2_costmap_2d", "nav2_costmap_2d::Layer"};

  /**
   * @brief Get parameters for node
   */
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
  std::vector<std::string> filter_names_;
  std::vector<std::string> filter_types_;
  double resolution_{0};
  std::string robot_base_frame_;   ///< The frame_id of the robot base
  double robot_radius_;
  bool rolling_window_{false};     ///< Whether to use a rolling window version of the costmap
  bool track_unknown_space_{false};
  double transform_tolerance_{0};  ///< The timeout before transform errors

  bool is_lifecycle_follower_{true};     ///< whether is a child-LifecycleNode or an independent node

  // Derived parameters
  bool use_radius_{false};
  std::vector<geometry_msgs::msg::Point> unpadded_footprint_;
  std::vector<geometry_msgs::msg::Point> padded_footprint_;

  std::unique_ptr<ClearCostmapService> clear_costmap_service_;

  // Dynamic parameters handler
  OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_2D_ROS_HPP_
