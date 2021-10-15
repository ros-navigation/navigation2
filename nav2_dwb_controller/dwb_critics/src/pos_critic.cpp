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
#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/pos_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using std::placeholders::_1;
PLUGINLIB_EXPORT_CLASS(dwb_critics::PosCritic, dwb_core::TrajectoryCritic)
#define PI 3.14159265
namespace dwb_critics
{

void PosCritic::onInit()
{

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".sum_scores", rclcpp::ParameterValue(false));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".sum_scores", sum_scores_);


  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".scale", rclcpp::ParameterValue(0.1));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);
  subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "worldpos", 10, std::bind(&PosCritic::topic_callback, this, _1));

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".decay", rclcpp::ParameterValue(0.9));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".decay", decay_);

  length_ = 0.0;
  target_x = 0.0;
  target_y = 0.0;
}

double PosCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0;
  for (unsigned int i = 0; i < traj.poses.size(); ++i) {
    double pose_score = scorePose(traj.poses[i]);
    // Optimized/branchless version of if (sum_scores_) score += pose_score,
    // else score = pose_score;
    score = static_cast<double>(sum_scores_) * score + pose_score * scale_ * length_;
  }
  return score;
}

double PosCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  if(length_<0.1)
    return 0;
  double x = pose.x - target_x;
  double y = pose.y - target_y;
  return sqrt(x*x + y*y);
  //return abs(pose.theta - angle_);

}

void PosCritic::debrief(const nav_2d_msgs::msg::Twist2D & twist)
{
  (void)twist;
  length_ *= decay_;
}
void PosCritic::topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  target_x = msg->pose.position.x;
  target_y = msg->pose.position.y;
  length_ = 1;
  
}

}  // namespace dwb_critics
