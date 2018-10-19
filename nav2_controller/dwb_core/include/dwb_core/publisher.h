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

#ifndef DWB_CORE__PUBLISHER_H_
#define DWB_CORE__PUBLISHER_H_

#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dwb_core/common_types.h"
#include "dwb_core/trajectory_critic.h"
#include "dwb_msgs/msg/local_plan_evaluation.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace dwb_core
{

/**
 * @class DWBPublisher
 * @brief Consolidation of all the publishing logic for the DWB Local Planner.
 *
 * Right now, it can publish
 *   1) The Global Plan (as passed in using setPath)
 *   2) The Local Plan (after it is calculated)
 *   3) The Transformed Global Plan (since it may be different than the global)
 *   4) The Full LocalPlanEvaluation
 *   5) Markers representing the different trajectories evaluated
 *   6) The CostGrid (in the form of a complex PointCloud2)
 */
class DWBPublisher
{
public:
  /**
   * @brief Load the parameters and advertise topics as needed
   * @param nh NodeHandle to load parameters from
   */
  void initialize(const std::shared_ptr<rclcpp::Node> & nh);

  /**
   * @brief Does the publisher require that the LocalPlanEvaluation be saved
   * @return True if the Evaluation is needed to publish either directly or as trajectories
   */
  bool shouldRecordEvaluation() {return publish_evaluation_ || publish_trajectories_;}

  /**
   * @brief If the pointer is not null, publish the evaluation and trajectories as needed
   */
  void publishEvaluation(std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results);
  void publishLocalPlan(
    const std_msgs::msg::Header & header,
    const dwb_msgs::msg::Trajectory2D & traj);
  void publishCostGrid(
    const CostmapROSPtr costmap_ros,
    const std::vector<TrajectoryCritic::Ptr> critics);
  void publishGlobalPlan(const nav_2d_msgs::msg::Path2D plan);
  void publishTransformedPlan(const nav_2d_msgs::msg::Path2D plan);
  void publishLocalPlan(const nav_2d_msgs::msg::Path2D plan);

protected:
  void publishTrajectories(const dwb_msgs::msg::LocalPlanEvaluation & results);

  // Helper function for publishing other plans
  void publishGenericPlan(
    const nav_2d_msgs::msg::Path2D plan,
    rclcpp::Publisher<nav_msgs::msg::Path> & pub, bool flag);

  // Flags for turning on/off publishing specific components
  bool publish_evaluation_, publish_global_plan_, publish_transformed_, publish_local_plan_,
    publish_trajectories_;
  bool publish_cost_grid_pc_;

  // Previously published marker count for removing markers as needed
  int prev_marker_count_;

  // Publisher Objects
  std::shared_ptr<rclcpp::Publisher<dwb_msgs::msg::LocalPlanEvaluation>> eval_pub_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> global_pub_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> transformed_pub_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> local_pub_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud>> cost_grid_pc_pub_;

  std::shared_ptr<rclcpp::Node> nh_;
};

}  // namespace dwb_core

#endif  // DWB_CORE__PUBLISHER_H_
