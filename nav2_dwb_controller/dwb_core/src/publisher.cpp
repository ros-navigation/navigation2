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

#include "dwb_core/publisher.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "nav_2d_utils/conversions.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::max;
using std::string;

namespace dwb_core
{

DWBPublisher::DWBPublisher(nav2_util::LifecycleNode::SharedPtr node)
: node_(node)
{
  node_->declare_parameter("publish_evaluation", rclcpp::ParameterValue(true));
  node_->declare_parameter("publish_global_plan", rclcpp::ParameterValue(true));
  node_->declare_parameter("publish_transformed_plan", rclcpp::ParameterValue(true));
  node_->declare_parameter("publish_local_plan", rclcpp::ParameterValue(true));
  node_->declare_parameter("publish_trajectories", rclcpp::ParameterValue(true));
  node_->declare_parameter("publish_cost_grid_pc", rclcpp::ParameterValue(false));
}

nav2_util::CallbackReturn
DWBPublisher::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  node_->get_parameter("publish_evaluation", publish_evaluation_);
  node_->get_parameter("publish_global_plan", publish_global_plan_);
  node_->get_parameter("publish_transformed_plan", publish_transformed_);
  node_->get_parameter("publish_local_plan", publish_local_plan_);
  node_->get_parameter("publish_trajectories", publish_trajectories_);
  node_->get_parameter("publish_cost_grid_pc", publish_cost_grid_pc_);

  eval_pub_ = node_->create_publisher<dwb_msgs::msg::LocalPlanEvaluation>("evaluation", 1);
  global_pub_ = node_->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  transformed_pub_ = node_->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 1);
  cost_grid_pc_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("cost_cloud", 1);

  prev_marker_count_ = 0;

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWBPublisher::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  eval_pub_->on_activate();
  global_pub_->on_activate();
  transformed_pub_->on_activate();
  local_pub_->on_activate();
  marker_pub_->on_activate();
  cost_grid_pc_pub_->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWBPublisher::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  eval_pub_->on_deactivate();
  global_pub_->on_deactivate();
  transformed_pub_->on_deactivate();
  local_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  cost_grid_pc_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWBPublisher::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  eval_pub_.reset();
  global_pub_.reset();
  transformed_pub_.reset();
  local_pub_.reset();
  marker_pub_.reset();
  cost_grid_pc_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void
DWBPublisher::publishEvaluation(std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results)
{
  if (results == nullptr) {return;}

  if (publish_evaluation_) {
    eval_pub_->publish(*results);
  }

  publishTrajectories(*results);
}

void
DWBPublisher::publishTrajectories(const dwb_msgs::msg::LocalPlanEvaluation & results)
{
  if (!publish_trajectories_) {return;}
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;

  if (results.twists.size() == 0) {return;}

  geometry_msgs::msg::Point pt;

  m.header = results.header;
  m.type = m.LINE_STRIP;
  m.pose.orientation.w = 1;
  m.scale.x = 0.002;
  m.color.a = 1.0;

  double best_cost = results.twists[results.best_index].total;
  double worst_cost = results.twists[results.worst_index].total;

  unsigned currentValidId = 0;
  unsigned currentInvalidId = 0;
  string validNamespace("ValidTrajectories");
  string invalidNamespace("InvalidTrajectories");
  for (unsigned int i = 0; i < results.twists.size(); i++) {
    const dwb_msgs::msg::TrajectoryScore & twist = results.twists[i];
    double displayLevel = (twist.total - best_cost) / (worst_cost - best_cost);
    if (twist.total >= 0) {
      m.color.r = displayLevel;
      m.color.g = 1.0 - displayLevel;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = validNamespace;
      m.id = currentValidId;
      ++currentValidId;
    } else {
      m.color.r = 0;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = 1.0;
      m.ns = invalidNamespace;
      m.id = currentInvalidId;
      ++currentInvalidId;
    }
    m.points.clear();
    for (unsigned int j = 0; j < twist.traj.poses.size(); ++j) {
      pt.x = twist.traj.poses[j].x;
      pt.y = twist.traj.poses[j].y;
      pt.z = 0;
      m.points.push_back(pt);
    }
    ma.markers.push_back(m);
  }
  addDeleteMarkers(ma, currentValidId, validNamespace);
  addDeleteMarkers(ma, currentInvalidId, invalidNamespace);
  prev_marker_count_ = max(currentValidId, currentInvalidId);
  marker_pub_->publish(ma);
}

void
DWBPublisher::publishLocalPlan(
  const std_msgs::msg::Header & header,
  const dwb_msgs::msg::Trajectory2D & traj)
{
  if (!publish_local_plan_) {return;}

  nav_msgs::msg::Path path =
    nav_2d_utils::poses2DToPath(traj.poses, header.frame_id, header.stamp);
  local_pub_->publish(path);
}

void
DWBPublisher::publishCostGrid(
  const CostmapROSPtr costmap_ros,
  const std::vector<TrajectoryCritic::Ptr> critics)
{
  if (!publish_cost_grid_pc_) {return;}

  sensor_msgs::msg::PointCloud cost_grid_pc;
  cost_grid_pc.header.frame_id = costmap_ros->getGlobalFrameID();
  cost_grid_pc.header.stamp = node_->now();

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros->getCostmap();
  double x_coord, y_coord;
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  cost_grid_pc.points.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      costmap->mapToWorld(cx, cy, x_coord, y_coord);
      cost_grid_pc.points[i].x = x_coord;
      cost_grid_pc.points[i].y = y_coord;
      i++;
    }
  }

  sensor_msgs::msg::ChannelFloat32 totals;
  totals.name = "total_cost";
  totals.values.resize(size_x * size_y, 0.0);

  for (TrajectoryCritic::Ptr critic : critics) {
    unsigned int channel_index = cost_grid_pc.channels.size();
    critic->addGridScores(cost_grid_pc);
    if (channel_index == cost_grid_pc.channels.size()) {
      // No channels were added, so skip to next critic
      continue;
    }
    double scale = critic->getScale();
    for (i = 0; i < size_x * size_y; i++) {
      totals.values[i] += cost_grid_pc.channels[channel_index].values[i] * scale;
    }
  }
  cost_grid_pc.channels.push_back(totals);

  // TODO(crdelsey): convert pc to pc2
  // sensor_msgs::msg::PointCloud2 cost_grid_pc2;
  // convertPointCloudToPointCloud2(cost_grid_pc, cost_grid_pc2);
  cost_grid_pc_pub_->publish(cost_grid_pc);
}

void
DWBPublisher::publishGlobalPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *global_pub_, publish_global_plan_);
}

void
DWBPublisher::publishTransformedPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *transformed_pub_, publish_transformed_);
}

void
DWBPublisher::publishLocalPlan(const nav_2d_msgs::msg::Path2D plan)
{
  publishGenericPlan(plan, *local_pub_, publish_local_plan_);
}

void
DWBPublisher::publishGenericPlan(
  const nav_2d_msgs::msg::Path2D plan,
  rclcpp::Publisher<nav_msgs::msg::Path> & pub, bool flag)
{
  if (!flag) {return;}
  nav_msgs::msg::Path path = nav_2d_utils::pathToPath(plan);
  pub.publish(path);
}

void
DWBPublisher::addDeleteMarkers(
  visualization_msgs::msg::MarkerArray & ma,
  unsigned startingId,
  string & ns
)
{
  visualization_msgs::msg::Marker m;
  m.action = m.DELETE;
  m.ns = ns;
  for (unsigned i = startingId; i < prev_marker_count_; i++) {
    m.id = i;
    ma.markers.push_back(m);
  }
}

}  // namespace dwb_core
