// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020, Samsung Research America
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

#ifndef nav2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_HPP_
#define nav2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_ceres_costaware_smoother/smoother_cost_function.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace nav2_ceres_costaware_smoother
{

/**
 * @class nav2_smac_planner::Smoother
 * @brief A Conjugate Gradient 2D path smoother implementation
 */
class Smoother
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Smoother
   */
  Smoother() {}

  /**
   * @brief A destructor for nav2_smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param params OptimizerParam struct
   */
  void initialize(const OptimizerParams params)
  {
    _debug = params.debug;
    // General Params

    // 2 most valid options: STEEPEST_DESCENT, NONLINEAR_CONJUGATE_GRADIENT
    _options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    _options.line_search_type = ceres::WOLFE;
    _options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
    _options.line_search_interpolation_type = ceres::CUBIC;

    _options.linear_solver_type = ceres::DENSE_QR;

    _options.max_num_iterations = params.max_iterations;
    _options.max_solver_time_in_seconds = params.max_time;

    _options.function_tolerance = params.fn_tol;
    _options.gradient_tolerance = params.gradient_tol;
    _options.parameter_tolerance = params.param_tol;

    _options.min_line_search_step_size = params.advanced.min_line_search_step_size;
    _options.max_num_line_search_step_size_iterations =
      params.advanced.max_num_line_search_step_size_iterations;
    _options.line_search_sufficient_function_decrease =
      params.advanced.line_search_sufficient_function_decrease;
    _options.max_line_search_step_contraction = params.advanced.max_line_search_step_contraction;
    _options.min_line_search_step_contraction = params.advanced.min_line_search_step_contraction;
    _options.max_num_line_search_direction_restarts =
      params.advanced.max_num_line_search_direction_restarts;
    _options.line_search_sufficient_curvature_decrease =
      params.advanced.line_search_sufficient_curvature_decrease;
    _options.max_line_search_step_expansion = params.advanced.max_line_search_step_expansion;

    if (_debug) {
      _options.minimizer_progress_to_stdout = true;
      _options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    } else {
      _options.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param costmap Pointer to minimal costmap
   * @param params parameters weights
   * @return If smoothing was successful
   */
  bool smooth(
    std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector2d & start_dir,
    const Eigen::Vector2d & end_dir,
    nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params)
  {
    //path always has at least 2 points
    CHECK(path.size() >= 2) << "Path must have at least 2 points";

    _options.max_solver_time_in_seconds = params.max_time;

    ceres::Problem problem;

    const double dir_change_half_length = params.dir_change_length/2;
    ceres::LossFunction* loss_function = NULL;
    std::vector<Eigen::Vector3d> pathOptim = path;
    std::vector<bool> optimized(path.size());
    int prelast_i = -1;
    int last_i = 0;
    double last_direction = 1; // to avoid compiler warning, actually was_reversing is always initialized during the first iteration
    bool last_had_direction_change = false;
    bool last_is_reversing = false;
    std::deque<std::pair<double, SmootherCostFunction *>> potential_dir_change_funcs;
    double last_segment_len = EPSILON_DOUBLE;
    double current_segment_len = 0;
    double potential_dir_change_funcs_len = 0;
    double len_since_dir_change = std::numeric_limits<double>::infinity();

    // Create residual blocks
    for (int i = 0; i < (int)pathOptim.size(); i++) {
      auto &pt = pathOptim[i];
      bool direction_change = false;
      if (i != (int)pathOptim.size()-1) {
        direction_change = pt[2]*last_direction < 0;
        last_direction = pt[2];

        // downsample if pose can be skipped (no forward/reverse direction change)
        if (i == 0 || (!direction_change && i > 1 && i < (int)pathOptim.size()-2 && (i - last_i) < params.input_downsampling_factor))
          continue;
      }

      // keep distance inequalities between poses (some might have been downsamled while others might not)
      current_segment_len += (pathOptim[i] - pathOptim[last_i]).block<2, 1>(0, 0).norm();
      potential_dir_change_funcs_len += current_segment_len;
      while (!potential_dir_change_funcs.empty() && potential_dir_change_funcs_len > dir_change_half_length) {
        potential_dir_change_funcs_len -= potential_dir_change_funcs.front().first;
        potential_dir_change_funcs.pop_front();
      }
      if (direction_change) {
        for (auto &f : potential_dir_change_funcs)
          f.second->setCostmapWeight(params.dir_change_costmap_weight);
        len_since_dir_change = 0;
        potential_dir_change_funcs_len = 0;
        potential_dir_change_funcs.clear();
      }

      optimized[i] = true;
      if (prelast_i != -1) {
        bool isDirChange = len_since_dir_change <= dir_change_half_length;
        SmootherCostFunction *cost_function = new SmootherCostFunction(
              path[last_i].template block<2, 1>(0, 0),
              (last_had_direction_change ? -1 : 1)*last_segment_len/current_segment_len, //(last_i - prelast_i)/(double)(i - last_i),
              last_is_reversing,
              costmap,
              params,
              isDirChange ? params.dir_change_costmap_weight : params.costmap_weight);
        problem.AddResidualBlock(cost_function->AutoDiff(), loss_function, pathOptim[last_i].data(), pt.data(), pathOptim[prelast_i].data());
        
        if (!isDirChange)
          potential_dir_change_funcs.emplace_back(current_segment_len, cost_function);
      }
      last_had_direction_change = direction_change;
      last_is_reversing = last_direction < 0;
      prelast_i = last_i;
      last_i = i;
      len_since_dir_change += current_segment_len;
      last_segment_len = std::max(EPSILON_DOUBLE, current_segment_len);
      current_segment_len = 0;
    }

    // first two and last two points are constant (to keep start and end direction)
    if (problem.NumParameterBlocks() > 4) {
      problem.SetParameterBlockConstant(pathOptim.front().data());
      problem.SetParameterBlockConstant(pathOptim[1].data());
      problem.SetParameterBlockConstant(pathOptim[pathOptim.size()-2].data());
      problem.SetParameterBlockConstant(pathOptim.back().data());

      // solve the problem
      ceres::Solver::Summary summary;
      ceres::Solve(_options, &problem, &summary);
      if (_debug) {
        RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "%s\nusable: %d, cost change ok: %d", summary.FullReport().c_str(),
                    summary.IsSolutionUsable(), summary.initial_cost - summary.final_cost >= 0.0);
      }
      if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost < 0.0) {
        return false;
      }
    }
    else
      RCLCPP_INFO(rclcpp::get_logger("smoother_server"), "Nothing to optimize");

    // Populate path, assign orientations, interpolate dropped poses
    path.resize(1); // keep the first point although not optimized
    path.back()[2] = atan2(start_dir[1], start_dir[0]);
    if (params.output_upsampling_factor > 1)
      path.reserve(params.output_upsampling_factor*(pathOptim.size() - 1) + 1);
    last_i = 0;
    prelast_i = -1;
    Eigen::Vector2d prelast_dir = start_dir;
    for (int i = 1; i <= (int)pathOptim.size(); i++)
      if (i == (int)pathOptim.size() || optimized[i]) {
        if (prelast_i != -1) {
          Eigen::Vector2d last_dir;
          auto &prelast = pathOptim[prelast_i];
          auto &last = pathOptim[last_i];
          auto &current = pathOptim[i];

          // Compute orientation of last
          if (i < (int)pathOptim.size()) {
            Eigen::Vector2d tangent_dir = tangentDir<double>(
              prelast.block<2, 1>(0, 0),
              last.block<2, 1>(0, 0),
              current.block<2, 1>(0, 0),
              prelast[2]*last[2]);
            last_dir = 
              tangent_dir.dot((current - last).block<2, 1>(0, 0)*last[2]) >= 0
                ? tangent_dir
                : -tangent_dir;
            last_dir.normalize();
          }
          else
            last_dir = end_dir;
          double last_angle = atan2(last_dir[1], last_dir[0]);

          // Interpolate poses between prelast and last
          int interp_cnt = (last_i - prelast_i)*params.output_upsampling_factor - 1;
          if (interp_cnt > 0) {
            Eigen::Vector2d lastp = last.block<2, 1>(0, 0);
            Eigen::Vector2d prelastp = prelast.block<2, 1>(0, 0);
            double dist = (lastp - prelastp).norm();
            Eigen::Vector2d p1 = prelastp + prelast_dir*dist*0.4*prelast[2];
            Eigen::Vector2d p2 = lastp - last_dir*dist*0.4*prelast[2];
            for (int j = 1; j <= interp_cnt; j++) {
              double interp = j/(double)(interp_cnt + 1);
              Eigen::Vector2d p = cubicBezier(prelastp, p1, p2, lastp, interp);
              path.emplace_back(p[0], p[1], 0.0);
            }
          }
          path.emplace_back(last[0], last[1], last_angle);

          // Assign orientations to interpolated points
          for (int j = path.size() - 1 - interp_cnt; j < (int)path.size() - 1; j++) {
            Eigen::Vector2d tangent_dir = tangentDir<double>(
              path[j-1].block<2, 1>(0, 0),
              path[j].block<2, 1>(0, 0),
              path[j+1].block<2, 1>(0, 0),
              1.0);
            tangent_dir = 
              tangent_dir.dot((path[j+1] - path[j]).block<2, 1>(0, 0)*prelast[2]) >= 0
                ? tangent_dir
                : -tangent_dir;
            path[j][2] = atan2(tangent_dir[1], tangent_dir[0]);
          }

          prelast_dir = last_dir;
        }
        prelast_i = last_i;
        last_i = i;
      }

    return true;
  }

  /*
    Piecewise cubic bezier curve as defined by Adobe in Postscript
    The two end points are p0 and p3
    Their associated control points are p1 and p2
  */
  Eigen::Vector2d cubicBezier(Eigen::Vector2d &p0, Eigen::Vector2d &p1, Eigen::Vector2d &p2, Eigen::Vector2d &p3, double mu)
  {
    Eigen::Vector2d a,b,c,p;

    c[0] = 3 * (p1[0] - p0[0]);
    c[1] = 3 * (p1[1] - p0[1]);
    b[0] = 3 * (p2[0] - p1[0]) - c[0];
    b[1] = 3 * (p2[1] - p1[1]) - c[1];
    a[0] = p3[0] - p0[0] - c[0] - b[0];
    a[1] = p3[1] - p0[1] - c[1] - b[1];

    p[0] = a[0] * mu * mu * mu + b[0] * mu * mu + c[0] * mu + p0[0];
    p[1] = a[1] * mu * mu * mu + b[1] * mu * mu + c[1] * mu + p0[1];

    return(p);
  }

private:
  bool _debug;
  ceres::Solver::Options _options;
};

}  // namespace nav2_smac_planner

#endif  // nav2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_HPP_
