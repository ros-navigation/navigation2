/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DWB_CORE__TRAJECTORY_CRITIC_HPP_
#define DWB_CORE__TRAJECTORY_CRITIC_HPP_

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_core
{
/**
 * @class TrajectoryCritic
 * @brief Evaluates a Trajectory2D to produce a score
 *
 * This class defines the plugin interface for the TrajectoryCritic which
 * gives scores to trajectories, where lower numbers are better, but negative
 * scores are considered invalid.
 *
 * The general lifecycle is
 *  1) initialize is called once at the beginning which in turn calls onInit.
 *       Derived classes may override onInit to load parameters as needed.
 *  2) prepare is called once before each set of trajectories.
 *       It is presumed that there are multiple trajectories that we want to evaluate,
 *       and there may be some shared work that can be done beforehand to optimize
 *       the scoring of each individual trajectory.
 *  3) scoreTrajectory is called once per trajectory and returns the score.
 *  4) debrief is called after each set of trajectories with the chosen trajectory.
 *       This can be used for stateful critics that monitor the trajectory through time.
 *
 *  Optionally, there is also a debugging mechanism for certain types of critics in the
 *  addCriticVisualization method. If the score for a trajectory depends on its relationship to
 *  the costmap, addCriticVisualization can provide that information to the dwb_core
 *  which will publish the grid scores as a PointCloud2.
 */
class TrajectoryCritic
{
public:
  using Ptr = std::shared_ptr<dwb_core::TrajectoryCritic>;

  virtual ~TrajectoryCritic() {}

  /**
   * @brief Initialize the critic with appropriate pointers and parameters
   *
   * The name and costmap are stored as member variables.
   * A NodeHandle is created using the combination of the parent namespace and the critic name
   *
   * @param name The name of this critic
   * @param parent_namespace The namespace of the planner
   * @param costmap_ros Pointer to the costmap
   */
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & name,
    const std::string & ns,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = nh;
    name_ = name;
    costmap_ros_ = costmap_ros;
    dwb_plugin_name_ = ns;
    if (!nh->has_parameter(dwb_plugin_name_ + "." + name_ + ".scale")) {
      nh->declare_parameter(
        dwb_plugin_name_ + "." + name_ + ".scale",
        rclcpp::ParameterValue(1.0));
    }
    nh->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);
    onInit();
  }
  virtual void onInit() {}

  /**
   * @brief Reset the state of the critic
   *
   * Reset is called when the planner receives a new global plan.
   * This can be used to discard information specific to one plan.
   */
  virtual void reset() {}

  /**
   * @brief Prior to evaluating any trajectories, look at contextual information constant across all trajectories
   *
   * Subclasses may overwrite. Return false in case there is any error.
   *
   * @param pose Current pose (costmap frame)
   * @param vel Current velocity
   * @param goal The final goal (costmap frame)
   * @param global_plan Transformed global plan in costmap frame, possibly cropped to nearby points
   */
  virtual bool prepare(
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &,
    const nav_2d_msgs::msg::Path2D &)
  {
    return true;
  }

  /**
   * @brief Return a raw score for the given trajectory.
   *
   * scores < 0 are considered invalid/errors, such as collisions
   * This is the raw score in that the scale should not be applied to it.
   */
  virtual double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) = 0;

  /**
   * @brief debrief informs the critic what the chosen cmd_vel was (if it cares)
   */
  virtual void debrief(const nav_2d_msgs::msg::Twist2D &) {}

  /**
   * @brief Add information to the given pointcloud for debugging costmap-grid based scores
   *
   * addCriticVisualization is an optional debugging mechanism for providing rich information
   * about the cost for certain trajectories. Some critics will have scoring mechanisms
   * wherein there will be some score for each cell in the costmap. This could be as
   * straightforward as the cost in the costmap, or it could be the number of cells away
   * from the goal pose.
   *
   * Prior to calling this, dwb_core will load the PointCloud's header and the points
   * in row-major order. The critic may then add a ChannelFloat to the channels member of the PC
   * with the same number of values as the points array. This information may then be converted
   * and published as a PointCloud2.
   *
   * @param pc PointCloud to add channels to
   */
  virtual void addCriticVisualization(std::vector<std::pair<std::string, std::vector<float>>> &) {}

  std::string getName()
  {
    return name_;
  }

  virtual double getScale() const {return scale_;}
  void setScale(const double scale) {scale_ = scale;}

protected:
  std::string name_;
  std::string dwb_plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  double scale_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
};

}  // namespace dwb_core

#endif  // DWB_CORE__TRAJECTORY_CRITIC_HPP_
