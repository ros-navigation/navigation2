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
#include "tf2/LinearMath/Quaternion.h"
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
  out.poses.reserve(mppi_path.x.shape(0));

  for (size_t i = 0; i < mppi_path.x.shape(0); ++i) {
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
}  // anonymous namespace

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
    logger_, "PathHugCritic initialised — weight %.2f, power %.2f",
    weight_, power_);
}

// ---------------------------------------------------------------------------
// main scoring loop
// ---------------------------------------------------------------------------
void PathHugCritic::score(CriticData & data)
{
  if (!enabled_ || data.path.x.shape(0) == 0 || weight_ == 0.0) {
    return;
  }

  /* cache the global plan if it changed */
  if (data.path.x.shape(0) != last_path_size_) {
    cached_path_ = mppiPathToNavMsgsPath(
      data.path, data.state.pose.header.frame_id);
    last_path_size_   = data.path.x.shape(0);
    closest_path_idx_ = 0;   // reset latch
  }

  const auto & trj  = data.trajectories;
  const size_t T    = trj.x.shape(0);  // trajectories
  const size_t P    = trj.x.shape(1);  // points per trajectory

  for (size_t i = 0; i < T; ++i) {
    float acc = 0.0f;
    size_t idx = closest_path_idx_;

    for (size_t j = 0; j < P; ++j) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id              = data.state.pose.header.frame_id;
      ps.pose.position.x              = trj.x(i, j);
      ps.pose.position.y              = trj.y(i, j);

      const double d = nav2_util::distanceFromPath(ps, cached_path_, &idx);
      acc += static_cast<float>(std::pow(d, power_));
    }

    closest_path_idx_ = idx;  // latch for next trajectory
    data.costs(i) += (acc / static_cast<float>(P)) * static_cast<float>(weight_);
  }
}

}  // namespace mppi::critics
