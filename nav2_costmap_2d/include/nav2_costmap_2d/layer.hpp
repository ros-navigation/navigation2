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
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__LAYER_HPP_
#define NAV2_COSTMAP_2D__LAYER_HPP_

#include <string>
#include <vector>
#include <unordered_set>

#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_costmap_2d
{
class LayeredCostmap;

/**
 * @class Layer
 * @brief Abstract class for layered costmap plugin implementations
 */
class Layer
{
public:
  /**
   * @brief A constructor
   */
  Layer();
  /**
   * @brief A destructor
   */
  virtual ~Layer() {}

  /**
   * @brief Initialization process of layer on startup
   */
  void initialize(
    LayeredCostmap * parent,
    std::string name,
    tf2_ros::Buffer * tf,
    const nav2::LifecycleNode::WeakPtr & node,
    rclcpp::CallbackGroup::SharedPtr callback_group);
  /** @brief Stop publishers. */
  virtual void deactivate() {}
  /** @brief Restart publishers if they've been stopped. */
  virtual void activate() {}
  /**
   * @brief Reset this costmap
   */
  virtual void reset() = 0;
  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() = 0;

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) = 0;

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  virtual void updateCosts(
    Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) = 0;

  /** @brief Implement this to make this layer match the size of the parent costmap. */
  virtual void matchSize() {}

  /** @brief LayeredCostmap calls this whenever the footprint there
   * changes (via LayeredCostmap::setFootprint()).  Override to be
   * notified of changes to the robot's footprint. */
  virtual void onFootprintChanged() {}
  /** @brief Get the name of the costmap layer */
  std::string getName() const
  {
    return name_;
  }

  /**
   * @brief Check to make sure all the data in the layer is up to date.
   *        If the layer is not up to date, then it may be unsafe to
   *        plan using the data from this layer, and the planner may
   *        need to know.
   *
   *        A layer's current state should be managed by the protected
   *        variable current_.
   * @return Whether the data in the layer is up to date.
   */
  bool isCurrent() const
  {
    return current_;
  }

  /**@brief Gets whether the layer is enabled. */
  bool isEnabled() const
  {
    return enabled_;
  }

  /** @brief Convenience function for layered_costmap_->getFootprint(). */
  const std::vector<geometry_msgs::msg::Point> & getFootprint() const;

  /** @brief Convenience functions for declaring ROS parameters */
  void declareParameter(
    const std::string & param_name,
    const rclcpp::ParameterValue & value);
  /** @brief Convenience functions for declaring ROS parameters */
  void declareParameter(
    const std::string & param_name,
    const rclcpp::ParameterType & param_type);
  /** @brief Convenience functions for declaring ROS parameters */
  bool hasParameter(const std::string & param_name);
  /** @brief Convenience functions for declaring ROS parameters */
  std::string getFullName(const std::string & param_name);

  /**
   * Joins the specified topic with the parent namespace of the layer node.
   * If the topic has an absolute path, it is returned instead.
   *
   * This is necessary for user defined relative topics to work as expected since costmap layers
   * add an additional `costmap_name` namespace to the topic.
   * For example:
   *   * User chosen namespace is `tb4`.
   *   * User chosen topic is `scan`.
   *   * Costmap node namespace will be `/tb4/global_costmap`.
   *   * Without this function, the topic would be `/tb4/global_costmap/scan`.
   *   * With this function, topic will be remapped to `/tb4/scan`.
   * Use global topic `/scan` if you do not wish the node namespace to apply
   *
   * @param topic the topic to parse
   */
  std::string joinWithParentNamespace(const std::string & topic);

protected:
  LayeredCostmap * layered_costmap_;
  std::string name_;
  tf2_ros::Buffer * tf_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};

  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   *
   * tf_, name_, and layered_costmap_ will all be set already when this is called.
   */
  virtual void onInitialize() {}

  bool current_;
  // Currently this var is managed by subclasses.
  // TODO(bpwilcox): make this managed by this class and/or container class.
  bool enabled_;

  // Names of the parameters declared on the ROS node
  std::unordered_set<std::string> local_params_;

private:
  std::vector<geometry_msgs::msg::Point> footprint_spec_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__LAYER_HPP_
