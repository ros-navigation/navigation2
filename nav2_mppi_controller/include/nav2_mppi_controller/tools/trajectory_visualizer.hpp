// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_

#include <Eigen/Dense>

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/trajectory.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"

namespace mppi
{

/**
 * @class mppi::TrajectoryVisualizer
 * @brief Visualizes trajectories for debugging
 */
class TrajectoryVisualizer
{
public:
  /**
    * @brief Constructor for mppi::TrajectoryVisualizer
    */
  TrajectoryVisualizer() = default;

  /**
    * @brief Configure trajectory visualizer
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param frame_id Frame to publish trajectories in
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    nav2::LifecycleNode::WeakPtr parent, const std::string & name,
    const std::string & frame_id, ParametersHandler * parameters_handler);

  /**
    * @brief Cleanup object on shutdown
    */
  void on_cleanup();

  /**
    * @brief Activate object
    */
  void on_activate();

  /**
    * @brief Deactivate object
    */
  void on_deactivate();

  /**
    * @brief Add an optimal trajectory to visualize
    * @param trajectory Optimal trajectory
    */
  void add(
    const Eigen::ArrayXXf & trajectory, const std::string & marker_namespace,
    const builtin_interfaces::msg::Time & cmd_stamp);

  /**
    * @brief Add candidate trajectories with costs to visualize
    * @param trajectories Candidate trajectories
    * @param total_costs Total cost array for each trajectory
    * @param individual_critics_cost Optional vector of (critic_name, cost_array) pairs for per-critic visualization
    * @param cmd_stamp Timestamp for the markers
    */
  void add(
    const models::Trajectories & trajectories,
    const Eigen::ArrayXf & total_costs,
    const std::vector<std::pair<std::string, Eigen::ArrayXf>> & individual_critics_cost = {},
    const builtin_interfaces::msg::Time & cmd_stamp = builtin_interfaces::msg::Time());

  /**
    * @brief Visualize all trajectory data in one call
    * @param plan Transformed plan to visualize
    * @param optimal_trajectory Optimal trajectory
    * @param control_sequence Control sequence for optimal trajectory
    * @param model_dt Model time step
    * @param stamp Timestamp for the visualization
    * @param costmap_ros Costmap ROS pointer
    * @param candidate_trajectories Generated candidate trajectories
    * @param costs Total costs for each trajectory
    * @param critic_costs Per-critic costs for each trajectory
    */
  void visualize(
    nav_msgs::msg::Path plan,
    const Eigen::ArrayXXf & optimal_trajectory,
    const models::ControlSequence & control_sequence,
    float model_dt,
    const builtin_interfaces::msg::Time & stamp,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const models::Trajectories & candidate_trajectories,
    const Eigen::ArrayXf & costs,
    const std::vector<std::pair<std::string, Eigen::ArrayXf>> & critic_costs);

  /**
    * @brief Visualize without optimizer (for testing)
    * @param plan Transformed plan to visualize
    */
  void visualize(nav_msgs::msg::Path plan);

  /**
    * @brief Reset object
    */
  void reset();

protected:
  /**
    * @brief Create trajectory markers with cost-based coloring
    * @param trajectories Set of trajectories to visualize
    * @param costs Cost array for each trajectory
    * @param ns Namespace for the markers
    * @param cmd_stamp Timestamp for the markers
    */
  void createTrajectoryMarkers(
    const models::Trajectories & trajectories,
    const Eigen::ArrayXf & costs,
    const std::string & ns,
    const builtin_interfaces::msg::Time & cmd_stamp);

  /**
    * @brief Create footprint markers from trajectory
    * @param trajectory Optimal trajectory
    * @param header Message header
    * @param costmap_ros Costmap ROS pointer for footprint and frame information
    * @return MarkerArray containing footprint polygons
    */
  visualization_msgs::msg::MarkerArray createFootprintMarkers(
    const Eigen::ArrayXXf & trajectory,
    const std_msgs::msg::Header & header,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  std::string frame_id_;
  nav2::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    trajectories_publisher_;
  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr transformed_path_pub_;
  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;
  nav2::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimal_footprints_pub_;
  nav2::Publisher<nav2_msgs::msg::Trajectory>::SharedPtr optimal_trajectory_msg_pub_;

  std::unique_ptr<nav_msgs::msg::Path> optimal_path_;
  std::unique_ptr<visualization_msgs::msg::MarkerArray> points_;
  int marker_id_ = 0;

  ParametersHandler * parameters_handler_;

  size_t trajectory_step_{0};
  size_t time_step_{0};
  bool publish_optimal_trajectory_{false};
  bool publish_trajectories_with_total_cost_{false};
  bool publish_trajectories_with_individual_cost_{false};
  bool publish_optimal_footprints_{false};
  bool publish_optimal_trajectory_msg_{false};
  bool publish_transformed_path_{false};
  bool publish_optimal_path_{false};
  int footprint_downsample_factor_{3};

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
