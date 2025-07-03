/******************************************************************************
 *  Copyright (c) 2025, Berkan Tali
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#include "nav2_mppi_controller/critics/path_hug_critic.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/path_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
//#include "tf2/LinearMath/Quaternion.h"          // tf2::Quaternion
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(mppi::critics::PathHugCritic, mppi::critics::CriticFunction)

namespace
{
// ---------------------------------------------------------------------------
// Helper: MPPI path → nav_msgs/Path
// ---------------------------------------------------------------------------
nav_msgs::msg::Path
mppiPathToNavMsgsPath(const mppi::models::Path & mppi_path,
                      const std::string & frame_id)
{
  nav_msgs::msg::Path out;
  out.header.frame_id = frame_id;

  const size_t path_len = static_cast<size_t>(mppi_path.x.rows());
  out.poses.reserve(path_len);

  for (size_t i = 0; i < path_len; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = out.header;
    ps.pose.position.x = mppi_path.x(i);
    ps.pose.position.y = mppi_path.y(i);
    ps.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, mppi_path.yaws(i));
    ps.pose.orientation = tf2::toMsg(q);

    out.poses.emplace_back(std::move(ps));
  }
  return out;
}
}  // namespace

namespace mppi::critics
{

// ---------------------------------------------------------------------------
// initialise parameters
// ---------------------------------------------------------------------------
void PathHugCritic::initialize()
{
  auto gp = parameters_handler_->getParamGetter(name_);
  gp(power_,  "power",  1.0);
  gp(weight_, "weight", 2.0);

  RCLCPP_INFO(
    logger_, "PathHugCritic initialised: weight %.2f  power %.2f",
    weight_, power_);
}

// ---------------------------------------------------------------------------
// main scoring loop
// ---------------------------------------------------------------------------
void PathHugCritic::score(CriticData & data)
{
  if (!enabled_ || data.path.x.rows() == 0 || weight_ == 0.0) {
    return;
  }

  // cache global plan once its size changes
  const size_t path_len = static_cast<size_t>(data.path.x.rows());
  if (path_len != last_path_size_) {
    cached_path_      = mppiPathToNavMsgsPath(
                           data.path, data.state.pose.header.frame_id);
    last_path_size_   = path_len;
    closest_path_idx_ = 0;
  }

  const auto & trj = data.trajectories;
  const size_t num_traj   = static_cast<size_t>(trj.x.rows());
  const size_t pts_per_tr = static_cast<size_t>(trj.x.cols());

  for (size_t i = 0; i < num_traj; ++i) {
    float accum = 0.0f;
    size_t idx  = closest_path_idx_;   // latch for iterative local search

    for (size_t j = 0; j < pts_per_tr; ++j) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = data.state.pose.header.frame_id;
      ps.pose.position.x = trj.x(i, j);
      ps.pose.position.y = trj.y(i, j);

      const double d = nav2_util::distanceFromPath(ps, cached_path_, &idx);
      accum += static_cast<float>(std::pow(d, power_));
    }

    closest_path_idx_ = idx;   // save for next trajectory
    data.costs(i) += (accum / static_cast<float>(pts_per_tr)) *
                     static_cast<float>(weight_);
  }
}

}  // namespace mppi::critics
