// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_controller/critics/path_align_critic.hpp"

namespace mppi::critics
{

void PathAlignCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(occupancy_check_min_distance_, "occupancy_check_min_distance", 2.0f);
  getParam(max_path_occupancy_ratio_, "max_path_occupancy_ratio", 0.07f);
  getParam(offset_from_furthest_, "offset_from_furthest", 20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(
    threshold_to_consider_,
    "threshold_to_consider", 0.5f);
  getParam(use_path_orientations_, "use_path_orientations", false);

  getParam(visualize_occupancy_check_distance_, "visualize_occupancy_check_distance", false);

  if (visualize_occupancy_check_distance_) {
    auto node = parent_.lock();
    if (node) {
      occupancy_check_dist_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "PathAlignCritic/occupancy_check_end_point", 1);
      occupancy_check_dist_pub_->on_activate();
    }
  }

  RCLCPP_INFO(
    logger_,
    "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
    power_, weight_);
}

void PathAlignCritic::score(CriticData & data)
{
  if (!enabled_ || data.state.local_path_length < threshold_to_consider_) {
    return;
  }

  // Only apply critic when trajectories reach far enough along the way path.
  // This ensures that path alignment is only considered when actually tracking the path
  // (e.g. not driving very slow or when first getting bearing w.r.t. the path)
  utils::setPathFurthestPointIfNotSet(data);

  const auto now = clock_->now();
  // Visualize furthest reached pose if enabled
  if (visualize_furthest_point_ && *data.furthest_reached_path_point > 0 &&
    furthest_point_pub_->get_subscription_count() > 0)
  {
    auto furthest_point = std::make_unique<geometry_msgs::msg::PoseStamped>();
    furthest_point->header.frame_id = costmap_ros_->getGlobalFrameID();
    furthest_point->header.stamp = now;
    furthest_point->pose.position.x = data.path.x(*data.furthest_reached_path_point);
    furthest_point->pose.position.y = data.path.y(*data.furthest_reached_path_point);
    furthest_point->pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, data.path.yaws(*data.furthest_reached_path_point));
    furthest_point->pose.orientation = tf2::toMsg(quat);
    furthest_point_pub_->publish(std::move(furthest_point));
  }

  if (*data.furthest_reached_path_point < offset_from_furthest_) {
    return;
  }

  const size_t batch_size = data.trajectories.x.rows();
  Eigen::ArrayXf cost(data.costs.rows());
  cost.setZero();

  // Find integrated arc-length distance along the path = total dist traveled along the path to each
  // path point loop until end of path, to guarantee don't truncate long trajectories when furthest
  // reached path is small (e.g. when all traj curve away from the path)
  const size_t path_segments_count = data.path.x.size() - 1;
  // initialize the occupancy check id to max, in case the entire path is within the distance
  size_t occupancy_check_distance_idx = path_segments_count;
  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  std::vector<utils::Pose2D> path(path_segments_count);
  float dx = 0.0f, dy = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    auto & pose = path[i - 1];
    pose.x = data.path.x(i - 1);
    pose.y = data.path.y(i - 1);
    pose.theta = data.path.yaws(i - 1);

    dx = data.path.x(i) - pose.x;
    dy = data.path.y(i) - pose.y;
    path_integrated_distances[i] = path_integrated_distances[i - 1] + sqrtf(dx * dx + dy * dy);

    // find the first path point that is further than
    //  max(occupancy_check_min_distance_, furthest_reached_path_point)
    if (occupancy_check_distance_idx == path_segments_count &&
      path_integrated_distances[i] > occupancy_check_min_distance_ &&
      i >= *data.furthest_reached_path_point)
    {
      occupancy_check_distance_idx = i;
    }
  }

  // Visualize occupancy check distance if enabled
  if (visualize_occupancy_check_distance_ &&
    occupancy_check_dist_pub_->get_subscription_count() > 0)
  {
    auto occupancy_check_point = std::make_unique<geometry_msgs::msg::PoseStamped>();
    occupancy_check_point->header.frame_id = costmap_ros_->getGlobalFrameID();
    occupancy_check_point->header.stamp = now;
    occupancy_check_point->pose.position.x = data.path.x(occupancy_check_distance_idx);
    occupancy_check_point->pose.position.y = data.path.y(occupancy_check_distance_idx);
    occupancy_check_point->pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, data.path.yaws(occupancy_check_distance_idx));
    occupancy_check_point->pose.orientation = tf2::toMsg(quat);
    occupancy_check_dist_pub_->publish(std::move(occupancy_check_point));
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the path
  // up to occupancy_check_min_distance_
  const float occupancy_check_distance_idx_flt = static_cast<float>(occupancy_check_distance_idx);
  utils::setPathCostsIfNotSet(data, costmap_ros_);
  std::vector<bool> & path_pts_valid = *data.path_pts_valid;
  float invalid_ctr = 0.0f;
  for (size_t i = 0; i < occupancy_check_distance_idx; i++) {
    if (!path_pts_valid[i]) {invalid_ctr += 1.0f;}
    if (invalid_ctr / occupancy_check_distance_idx_flt > max_path_occupancy_ratio_ &&
      invalid_ctr > 2.0f)
    {
      return;
    }
  }

  // Finish populating the path vector
  auto & final_pose = path[path_segments_count - 1];
  final_pose.x = data.path.x(path_segments_count - 1);
  final_pose.y = data.path.y(path_segments_count - 1);
  final_pose.theta = data.path.yaws(path_segments_count - 1);

  float summed_path_dist = 0.0f, dyaw = 0.0f;
  unsigned int num_samples = 0u;
  unsigned int path_pt = 0u;
  float traj_integrated_distance = 0.0f;

  int strided_traj_rows = data.trajectories.x.rows();
  int strided_traj_cols = floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
  int outer_stride = strided_traj_rows * trajectory_point_step_;
  // Get strided trajectory information
  const auto T_x = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.trajectories.x.data(),
    strided_traj_rows, strided_traj_cols, Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto T_y = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.trajectories.y.data(),
    strided_traj_rows, strided_traj_cols, Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto T_yaw = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(
    data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
    Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_sampled_size = T_x.cols();

  for (size_t t = 0; t < batch_size; ++t) {
    summed_path_dist = 0.0f;
    num_samples = 0u;
    traj_integrated_distance = 0.0f;
    path_pt = 0u;
    float Tx_m1 = T_x(t, 0);
    float Ty_m1 = T_y(t, 0);
    for (int p = 1; p < traj_sampled_size; p++) {
      const float Tx = T_x(t, p);
      const float Ty = T_y(t, p);
      dx = Tx - Tx_m1;
      dy = Ty - Ty_m1;
      Tx_m1 = Tx;
      Ty_m1 = Ty;
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      if (path_pts_valid[path_pt]) {
        const auto & pose = path[path_pt];
        dx = pose.x - Tx;
        dy = pose.y - Ty;
        num_samples++;
        if (use_path_orientations_) {
          dyaw = angles::shortest_angular_distance(pose.theta, T_yaw(t, p));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    if (num_samples > 0u) {
      cost(t) = summed_path_dist / static_cast<float>(num_samples);
    } else {
      cost(t) = 0.0f;
    }
  }

  if (power_ > 1u) {
    data.costs += (cost * weight_).pow(power_).eval();
  } else {
    data.costs += (cost * weight_).eval();
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathAlignCritic,
  mppi::critics::CriticFunction)
