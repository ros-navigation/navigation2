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

#ifndef NAV2_SMAC_PLANNER__TYPES_HPP_
#define NAV2_SMAC_PLANNER__TYPES_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_smac_planner
{

typedef std::pair<float, uint64_t> NodeHeuristicPair;

/**
 * @struct nav2_smac_planner::SearchInfo
 * @brief Search properties and penalties
 */
struct SearchInfo
{
  float minimum_turning_radius{8.0};
  float non_straight_penalty{1.05};
  float change_penalty{0.0};
  float reverse_penalty{2.0};
  float cost_penalty{2.0};
  float retrospective_penalty{0.015};
  float rotation_penalty{5.0};
  float analytic_expansion_ratio{3.5};
  float analytic_expansion_max_length{60.0};
  float analytic_expansion_max_cost{200.0};
  bool analytic_expansion_max_cost_override{false};
  std::string lattice_filepath;
  bool cache_obstacle_heuristic{false};
  bool allow_reverse_expansion{false};
  bool allow_primitive_interpolation{false};
  bool downsample_obstacle_heuristic{true};
  bool use_quadratic_cost_penalty{false};
};

/**
 * @struct nav2_smac_planner::SmootherParams
 * @brief Parameters for the smoother
 */
struct SmootherParams
{
  /**
   * @brief A constructor for nav2_smac_planner::SmootherParams
   */
  SmootherParams()
  : holonomic_(false)
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string & name)
  {
    std::string local_name = name + std::string(".smoother.");

    // Smoother params
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "tolerance", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "tolerance", tolerance_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "max_iterations", rclcpp::ParameterValue(1000));
    node->get_parameter(local_name + "max_iterations", max_its_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_data", rclcpp::ParameterValue(0.2));
    node->get_parameter(local_name + "w_data", w_data_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_smooth", rclcpp::ParameterValue(0.3));
    node->get_parameter(local_name + "w_smooth", w_smooth_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "do_refinement", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "do_refinement", do_refinement_);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "refinement_num", rclcpp::ParameterValue(2));
    node->get_parameter(local_name + "refinement_num", refinement_num_);
  }

  double tolerance_;
  int max_its_;
  double w_data_;
  double w_smooth_;
  bool holonomic_;
  bool do_refinement_;
  int refinement_num_;
};

/**
 * @struct nav2_smac_planner::TurnDirection
 * @brief A struct with the motion primitive's direction embedded
 */
enum struct TurnDirection
{
  UNKNOWN = 0,
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  REVERSE = 4,
  REV_LEFT = 5,
  REV_RIGHT = 6
};

/**
 * @struct nav2_smac_planner::MotionPose
 * @brief A struct for poses in motion primitives
 */
struct MotionPose
{
  /**
   * @brief A constructor for nav2_smac_planner::MotionPose
   */
  MotionPose() {}

  /**
   * @brief A constructor for nav2_smac_planner::MotionPose
   * @param x X pose
   * @param y Y pose
   * @param theta Angle of pose
   * @param TurnDirection Direction of the primitive's turn
   */
  MotionPose(const float & x, const float & y, const float & theta, const TurnDirection & turn_dir)
  : _x(x), _y(y), _theta(theta), _turn_dir(turn_dir)
  {}

  MotionPose operator-(const MotionPose & p2)
  {
    return MotionPose(
      this->_x - p2._x, this->_y - p2._y, this->_theta - p2._theta, TurnDirection::UNKNOWN);
  }

  float _x;
  float _y;
  float _theta;
  TurnDirection _turn_dir;
};

typedef std::vector<MotionPose> MotionPoses;

/**
 * @struct nav2_smac_planner::LatticeMetadata
 * @brief A struct of all lattice metadata
 */
struct LatticeMetadata
{
  float min_turning_radius;
  float grid_resolution;
  unsigned int number_of_headings;
  std::vector<float> heading_angles;
  unsigned int number_of_trajectories;
  std::string motion_model;
};

/**
 * @struct nav2_smac_planner::MotionPrimitive
 * @brief A struct of all motion primitive data
 */
struct MotionPrimitive
{
  unsigned int trajectory_id;
  float start_angle;
  float end_angle;
  float turning_radius;
  float trajectory_length;
  float arc_length;
  float straight_length;
  bool left_turn;
  MotionPoses poses;
};

typedef std::vector<MotionPrimitive> MotionPrimitives;
typedef std::vector<MotionPrimitive *> MotionPrimitivePtrs;

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__TYPES_HPP_
