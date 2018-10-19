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
#ifndef NAV2_COSTMAP_2D__COSTMAP_2D_ROS_H_
#define NAV2_COSTMAP_2D__COSTMAP_2D_ROS_H_

#include <nav2_costmap_2d/layered_costmap.h>
#include <nav2_costmap_2d/layer.h>
#include <nav2_costmap_2d/costmap_2d_publisher.h>
#include <nav2_costmap_2d/footprint.h>
#include <geometry_msgs/msg/polygon.h>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <pluginlib/class_loader.hpp>
#include <xmlrpcpp/XmlRpcValue.h>
#include <tf2/transform_datatypes.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_dynamic_params/dynamic_params_validator.hpp"
#include "nav2_dynamic_params/dynamic_params_client.hpp"

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct * a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray * a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace nav2_costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS : public rclcpp::Node
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(const std::string & name, tf2_ros::Buffer & tf);
  ~Costmap2DROS();

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
  bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose) const;

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

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap(). */
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
  void getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint) const;

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
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points);

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
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::msg::Polygon::SharedPtr footprint);

protected:
  LayeredCostmap * layered_costmap_;
  std::string name_;
  tf2_ros::Buffer & tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;  ///< timeout before transform errors

private:
  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  void readFootprintFromConfig(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  void resetOldParameters(rclcpp::Node::SharedPtr nh);

  void setPluginParams(rclcpp::Node::SharedPtr nh);

  void reconfigureCB(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  void movementCB();

  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_;
  bool stop_updates_, initialized_, stopped_, robot_stopped_;
  std::thread * map_update_thread_;  ///< @brief A thread for updating the map
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_publish_;
  rclcpp::Duration publish_cycle_;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  geometry_msgs::msg::PoseStamped old_pose_;
  Costmap2DPublisher * publisher_;

  nav2_dynamic_params::DynamicParamsValidator * param_validator_;
  nav2_dynamic_params::DynamicParamsClient * dynamic_param_client_;

  std::recursive_mutex configuration_mutex_;
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::SyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_;

  std::vector<geometry_msgs::msg::Point> unpadded_footprint_;
  std::vector<geometry_msgs::msg::Point> padded_footprint_;
  float footprint_padding_;

  rclcpp::Node::SharedPtr private_nh_;
};
// class Costmap2DROS
}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_2D_ROS_H
