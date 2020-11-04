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

#include "dwb_critics/map_grid.hpp"
#include <cmath>
#include <string>
#include <algorithm>
#include <memory>
#include "dwb_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

using std::abs;
using costmap_queue::CellData;

namespace dwb_critics
{

// Customization of the CostmapQueue validCellToQueue method
bool MapGridCritic::MapGridQueue::validCellToQueue(const costmap_queue::CellData & /*cell*/)
{
  return true;
}

void MapGridCritic::onInit()
{
  costmap_ = costmap_ros_->getCostmap();
  queue_ = std::make_shared<MapGridQueue>(*costmap_, *this);

  // Always set to true, but can be overriden by subclasses
  stop_on_failure_ = true;

  nav2_util::declare_parameter_if_not_declared(
    nh_,
    dwb_plugin_name_ + "." + name_ + ".aggregation_type",
    rclcpp::ParameterValue(std::string("last")));

  std::string aggro_str;
  nh_->get_parameter(dwb_plugin_name_ + "." + name_ + ".aggregation_type", aggro_str);
  std::transform(aggro_str.begin(), aggro_str.end(), aggro_str.begin(), ::tolower);
  if (aggro_str == "last") {
    aggregationType_ = ScoreAggregationType::Last;
  } else if (aggro_str == "sum") {
    aggregationType_ = ScoreAggregationType::Sum;
  } else if (aggro_str == "product") {
    aggregationType_ = ScoreAggregationType::Product;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "MapGridCritic"), "aggregation_type parameter \"%s\" invalid. Using Last.",
      aggro_str.c_str());
    aggregationType_ = ScoreAggregationType::Last;
  }
}

void MapGridCritic::setAsObstacle(unsigned int index)
{
  cell_values_[index] = obstacle_score_;
}

void MapGridCritic::reset()
{
  queue_->reset();
  cell_values_.resize(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY());
  obstacle_score_ = static_cast<double>(cell_values_.size());
  unreachable_score_ = obstacle_score_ + 1.0;
  std::fill(cell_values_.begin(), cell_values_.end(), unreachable_score_);
}

void MapGridCritic::propogateManhattanDistances()
{
  while (!queue_->isEmpty()) {
    costmap_queue::CellData cell = queue_->getNextCell();
    cell_values_[cell.index_] = CellData::absolute_difference(cell.src_x_, cell.x_) +
      CellData::absolute_difference(cell.src_y_, cell.y_);
  }
}

double MapGridCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double score = 0.0;
  unsigned int start_index = 0;
  if (aggregationType_ == ScoreAggregationType::Product) {
    score = 1.0;
  } else if (aggregationType_ == ScoreAggregationType::Last && !stop_on_failure_) {
    start_index = traj.poses.size() - 1;
  }
  double grid_dist;

  for (unsigned int i = start_index; i < traj.poses.size(); ++i) {
    grid_dist = scorePose(traj.poses[i]);
    if (stop_on_failure_) {
      if (grid_dist == obstacle_score_) {
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
      } else if (grid_dist == unreachable_score_) {
        throw dwb_core::
              IllegalTrajectoryException(name_, "Trajectory Hits Unreachable Area.");
      }
    }

    switch (aggregationType_) {
      case ScoreAggregationType::Last:
        score = grid_dist;
        break;
      case ScoreAggregationType::Sum:
        score += grid_dist;
        break;
      case ScoreAggregationType::Product:
        if (score > 0) {
          score *= grid_dist;
        }
        break;
    }
  }

  return score;
}

double MapGridCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  unsigned int cell_x, cell_y;
  // we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if (!costmap_->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  return getScore(cell_x, cell_y);
}

void MapGridCritic::addCriticVisualization(sensor_msgs::msg::PointCloud & pc)
{
  sensor_msgs::msg::ChannelFloat32 grid_scores;
  grid_scores.name = name_;

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  grid_scores.values.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++) {
    for (unsigned int cx = 0; cx < size_x; cx++) {
      grid_scores.values[i] = getScore(cx, cy);
      i++;
    }
  }
  pc.channels.push_back(grid_scores);
}

}  // namespace dwb_critics
