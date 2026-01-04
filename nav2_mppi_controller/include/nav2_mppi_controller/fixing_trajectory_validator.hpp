// include/nav2_mppi_controller/validators/auto_fixing_trajectory_validator.hpp
#ifndef NAV2_MPPI_CONTROLLER__AUTO_FIXING_TRAJECTORY_VALIDATOR_HPP_
#define NAV2_MPPI_CONTROLLER__AUTO_FIXING_TRAJECTORY_VALIDATOR_HPP_

#include "nav2_mppi_controller/optimal_trajectory_validator.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include <optional>

namespace mppi {

class AutoFixingTrajectoryValidator : public OptimalTrajectoryValidator
{
public:
  using Ptr = std::shared_ptr<AutoFixingTrajectoryValidator>;
  AutoFixingTrajectoryValidator() = default;
  ~AutoFixingTrajectoryValidator() override = default;

  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
    ParametersHandler * param_handler,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const models::OptimizerSettings & settings) override;

  ValidationResult validateTrajectory(
    const Eigen::ArrayXXf & optimal_trajectory,
    const models::ControlSequence & control_sequence,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::Pose & goal) override;

  // Po SUCCESS można pobrać naprawioną trajektorię (jeśli była korygowana)
  bool hasFixedTrajectory() const { return has_fixed_; }
  const Eigen::ArrayXXf & getFixedTrajectory() const { return fixed_traj_; }

private:
  // --- narzędzia kolizyjne / naprawcze ---
  bool isCollisionAt(double x, double y, double theta) const;
  bool isSafe(double x, double y, double theta) const;
  double clearanceAt(double x, double y) const;
  std::optional<Eigen::Vector2d> gradientAt(double x, double y) const;

  bool tryRepair(Eigen::ArrayXXf & traj, int start_idx, int end_idx) const;
  void laplacianSmoothXY(Eigen::ArrayXXf & traj, const std::vector<bool> &locked, int iters) const;
  void smoothYaw(Eigen::ArrayXXf & traj) const;

  // --- stan ---
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>> fp_checker_;

  // --- parametry ---
  bool consider_footprint_{true};
  double safe_clearance_{0.12};
  double step_gain_{0.6};
  double max_step_{0.05};
  double max_total_disp_{0.25};
  int    max_iters_{6};
  int    smooth_iters_{2};
  float model_dt_{0.f};
  float auto_fix_ttc_window_{0.8f}; 

  // --- wynik naprawy ---
  mutable bool has_fixed_{false};
  mutable Eigen::ArrayXXf fixed_traj_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__AUTO_FIXING_TRAJECTORY_VALIDATOR_HPP_
