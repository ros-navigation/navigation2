// Copyright (c) 2022, Samsung Research America
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
// limitations under the License. Reserved.

#include <vector>
#include <memory>
#include "nav2_smoother/savitzky_golay_smoother.hpp"
#include "nav2_core/smoother_exceptions.hpp"

namespace nav2_smoother
{
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT
using nav2_util::PathSegment;
using rcl_interfaces::msg::ParameterType;

void SavitzkyGolaySmoother::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, nav2::TransformBuffer::SharedPtr/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/*costmap_sub*/,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  node_ = parent;
  plugin_name_ = name;

  do_refinement_ = node->declare_or_get_parameter(
    name + ".do_refinement", true);
  refinement_num_ = node->declare_or_get_parameter(
    name + ".refinement_num", 2);
  enforce_path_inversion_ = node->declare_or_get_parameter(
    name + ".enforce_path_inversion", true);
  window_size_ = node->declare_or_get_parameter(
    name + ".window_size", 7);
  poly_order_ = node->declare_or_get_parameter(
    name + ".poly_order", 3);

  if (window_size_ % 2 == 0 || window_size_ <= 2) {
    throw nav2_core::SmootherException(
            "Savitzky-Golay Smoother requires an odd window size of 3 or greater");
  }
  half_window_size_ = (window_size_ - 1) / 2;
  calculateCoefficients();
}

void SavitzkyGolaySmoother::cleanup()
{
}

void SavitzkyGolaySmoother::activate()
{
  auto node = node_.lock();
  // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &SavitzkyGolaySmoother::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &SavitzkyGolaySmoother::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void SavitzkyGolaySmoother::deactivate()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}

// For more details on calculating Savitzky–Golay filter coefficients,
// see: https://www.colmryan.org/posts/savitsky_golay/
void SavitzkyGolaySmoother::calculateCoefficients()
{
  // We construct the Vandermonde matrix here
  Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(
    window_size_, -half_window_size_,
    half_window_size_);
  Eigen::MatrixXd x = Eigen::MatrixXd::Ones(window_size_, poly_order_ + 1);
  for (int i = 1; i <= poly_order_; i++) {
    x.col(i) = (x.col(i - 1).array() * v.array()).matrix();
  }
  // Compute the pseudoinverse of X by solving the least-squares problem X * C = I.
  // HouseholderQR factors X into an orthogonal matrix and an upper-triangular matrix,
  // then solves for the coefficient matrix C without explicitly inverting X.
  Eigen::MatrixXd coeff_mat =
    x.householderQr().solve(Eigen::MatrixXd::Identity(window_size_, window_size_));

  // Extract the smoothing coefficients
  sg_coeffs_ = coeff_mat.row(0).transpose();
}

bool SavitzkyGolaySmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.seconds();

  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;

  std::vector<PathSegment> path_segments{
    PathSegment{0u, static_cast<unsigned int>(path.poses.size() - 1)}};
  if (enforce_path_inversion_) {
    path_segments = nav2_util::findDirectionalPathSegments(path);
  }

  // Minimum point size to smooth is SG filter size + start + end
  unsigned int minimum_points = window_size_ + 2;
  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > minimum_points) {
      // Populate path segment
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.seconds() - duration_cast<duration<double>>(now - start).count();

      if (time_remaining <= 0.0) {
        RCLCPP_WARN(
          logger_,
          "Smoothing time exceeded allowed duration of %0.2f.", max_time.seconds());
        throw nav2_core::SmootherTimedOut("Smoothing time exceed allowed duration");
      }

      // Smooth path segment
      success = success && smoothImpl(curr_path_segment, reversing_segment);

      // Assemble the path changes to the main path
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

bool SavitzkyGolaySmoother::smoothImpl(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{
  const unsigned int & path_size = path.poses.size();

  // Convert PoseStamped to Eigen
  auto toEigenVec = [](const geometry_msgs::msg::PoseStamped & pose) -> Eigen::Vector2d {
      return {pose.pose.position.x, pose.pose.position.y};
    };

  auto applyFilterOverAxes =
    [&](std::vector<geometry_msgs::msg::PoseStamped> & plan_pts,
    const std::vector<Eigen::Vector2d> & init_plan_pts) -> void
    {
      // First point is fixed
      for (unsigned int idx = 1; idx != path_size - 1; idx++) {
        Eigen::Vector2d accum(0.0, 0.0);

        for (int j = -half_window_size_; j <= half_window_size_; j++) {
          int path_idx = std::clamp<int>(idx + j, 0, path_size - 1);
          accum += sg_coeffs_(j + half_window_size_) * init_plan_pts[path_idx];
        }
        plan_pts[idx].pose.position.x = accum.x();
        plan_pts[idx].pose.position.y = accum.y();
      }
    };

  std::vector<Eigen::Vector2d> initial_path_poses(path.poses.size());
  std::transform(
    path.poses.begin(), path.poses.end(),
    initial_path_poses.begin(), toEigenVec);
  applyFilterOverAxes(path.poses, initial_path_poses);

  // Let's do additional refinement, it shouldn't take more than a couple milliseconds
  if (do_refinement_) {
    for (int i = 0; i < refinement_num_; i++) {
      std::vector<Eigen::Vector2d> reined_initial_path_poses(path.poses.size());
      std::transform(
        path.poses.begin(), path.poses.end(),
        reined_initial_path_poses.begin(), toEigenVec);
      applyFilterOverAxes(path.poses, reined_initial_path_poses);
    }
  }

  nav2_util::updateApproximatePathOrientations(path, reversing_segment);
  return true;
}

rcl_interfaces::msg::SetParametersResult
SavitzkyGolaySmoother::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // window_size and poly_order constrain each other, so work out what the pair
  // would be after this batch is applied and validate them together
  int window_size = window_size_;
  int poly_order = poly_order_;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == plugin_name_ + ".window_size") {
      window_size = parameter.as_int();
    } else if (parameter.get_name() == plugin_name_ + ".poly_order") {
      poly_order = parameter.as_int();
    }
  }
  const bool window_size_valid =
    window_size % 2 == 1 && window_size >= 3 && window_size > poly_order;
  const bool poly_order_valid = poly_order >= 0 && poly_order < window_size;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (!mutex_.try_lock()) {
      RCLCPP_WARN(
        logger_, "Unable to change parameter '%s' while smoothing is in progress. "
        "Rejecting update.", param_name.c_str());
      result.successful = false;
      return result;
    }
    mutex_.unlock();
    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".refinement_num" && parameter.as_int() < 0) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %ld, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_int());
        result.successful = false;
      } else if (param_name == plugin_name_ + ".window_size" && !window_size_valid) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %d, "
          "it should be odd, at least 3, and greater than poly_order (%d). "
          "Ignoring parameter update.",
          param_name.c_str(), window_size, poly_order);
        result.successful = false;
      } else if (param_name == plugin_name_ + ".poly_order" && !poly_order_valid) {
        RCLCPP_WARN(
          logger_, "The value of parameter '%s' is incorrectly set to %d, "
          "it should be >=0 and less than window_size (%d). Ignoring parameter update.",
          param_name.c_str(), poly_order, window_size);
        result.successful = false;
      }
    }
  }
  return result;
}

void
SavitzkyGolaySmoother::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  bool reinit_coeffs = false;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == plugin_name_ + ".refinement_num") {
        refinement_num_ = parameter.as_int();
      } else if (param_name == plugin_name_ + ".window_size") {
        window_size_ = parameter.as_int();
        reinit_coeffs = true;
      } else if (param_name == plugin_name_ + ".poly_order") {
        poly_order_ = parameter.as_int();
        reinit_coeffs = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".do_refinement") {
        do_refinement_ = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".enforce_path_inversion") {
        enforce_path_inversion_ = parameter.as_bool();
      }
    }
  }

  // Window or polynomial order changed, so the filter coefficients must be rebuilt
  if (reinit_coeffs) {
    half_window_size_ = (window_size_ - 1) / 2;
    calculateCoefficients();
  }
}

}  // namespace nav2_smoother

#include "pluginlib/class_list_macros.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smoother::SavitzkyGolaySmoother, nav2_core::Smoother)
