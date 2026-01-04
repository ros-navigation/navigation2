// src/validators/auto_fixing_trajectory_validator.cpp
#include "nav2_mppi_controller/fixing_trajectory_validator.hpp"
#include <algorithm>
#include <cmath>

namespace mppi {

void AutoFixingTrajectoryValidator::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
  ParametersHandler * param_handler,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const models::OptimizerSettings & settings)
{
  OptimalTrajectoryValidator::initialize(parent, name, costmap, param_handler, tf_buffer, settings);
  auto node = node_.lock();
  auto get = param_handler_->getParamGetter(name_);

  model_dt_ = settings.model_dt;
  get(auto_fix_ttc_window_, "auto_fix_ttc_window", 0.8f);

  costmap_ = costmap_ros_->getCostmap();

  get(consider_footprint_, "consider_footprint", true);
  get(safe_clearance_, "auto_fix_safe_clearance", 0.12);
  get(step_gain_, "auto_fix_step_gain", 0.6);
  get(max_step_, "auto_fix_max_step", 0.05);
  get(max_total_disp_, "auto_fix_max_total_displacement", 0.25);
  get(max_iters_, "auto_fix_max_iters", 6);
  get(smooth_iters_, "auto_fix_smooth_iters", 2);

  // Clamping parametrów względem rozdzielczości costmapy – odporność na złe YAML
  const double r = costmap_->getResolution();
  auto clamp = [&](double &v, double lo, double hi) { v = std::clamp(v, lo, hi); };
  clamp(safe_clearance_, 0.5 * r, 1.5 * r);
  clamp(max_step_,       0.25 * r, 1.5 * r);
  clamp(max_total_disp_, 1.5 * r, 10.0 * r);
  step_gain_ = std::clamp(step_gain_, 0.2, 0.9);

  if (consider_footprint_) {
    fp_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>>(costmap_);
  }

  RCLCPP_INFO(node->get_logger(),
    "[%s] AutoFixingTrajectoryValidator initialized: safe=%.2f step_gain=%.2f "
    "max_step=%.2f max_disp=%.2f iters=%d smooth=%d lookahead=%.2fs (samples=%u)",
    name_.c_str(), safe_clearance_, step_gain_, max_step_, max_total_disp_,
    max_iters_, smooth_iters_, collision_lookahead_time_, traj_samples_to_evaluate_);
}

bool AutoFixingTrajectoryValidator::isCollisionAt(double x, double y, double theta) const
{
  if (consider_footprint_) {
    const double c = fp_checker_->footprintCostAtPose(
      x, y, theta, costmap_ros_->getRobotFootprint());
    // W tym Nav2 checkerze kolizja/OOB ➜ LETHAL (254). Traktuj też INSCRIBED (253) jako nieakceptowalne.
    return (c >= static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
  }

  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return true;  // poza mapą = niebezpieczne
  }
  const unsigned char c = costmap_->getCost(mx, my);
  return (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

double AutoFixingTrajectoryValidator::clearanceAt(double x, double y) const
{
  // Prosty promieniowy skan (jeśli masz EDT/SDF – podmień)
  static const int K = 12;
  static const double MAX_R = 0.6;
  double best = MAX_R;
  for (int k = 0; k < K; ++k) {
    const double ang = 2.0 * M_PI * k / K;
    const double cs = std::cos(ang), sn = std::sin(ang);
    double r = 0.0;
    while (r < MAX_R) {
      const double px = x + r * cs;
      const double py = y + r * sn;
      unsigned int mx, my;
      if (!costmap_->worldToMap(px, py, mx, my)) break;
      const unsigned char c = costmap_->getCost(mx, my);
      if (c == nav2_costmap_2d::LETHAL_OBSTACLE ||
          c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        best = std::min(best, r);
        break;
      }
      r += costmap_->getResolution();
    }
  }
  return best;
}

bool AutoFixingTrajectoryValidator::isSafe(double x, double y, double theta) const
{
  if (isCollisionAt(x, y, theta)) return false;
  return clearanceAt(x, y) >= safe_clearance_;
}

std::optional<Eigen::Vector2d> AutoFixingTrajectoryValidator::gradientAt(double x, double y) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return std::nullopt;

  auto sample = [&](int dx, int dy) -> double {
    const int ix = static_cast<int>(mx) + dx;
    const int iy = static_cast<int>(my) + dy;
    if (ix < 0 || iy < 0 ||
        ix >= static_cast<int>(costmap_->getSizeInCellsX()) ||
        iy >= static_cast<int>(costmap_->getSizeInCellsY())) {
      return 0.0;
    }
    return static_cast<double>(costmap_->getCost(ix, iy));
  };

  const double cx1 = sample(+1, 0), cx0 = sample(-1, 0);
  const double cy1 = sample(0, +1), cy0 = sample(0, -1);
  const double dx_cost = (cx1 - cx0) / (2.0 * costmap_->getResolution());
  const double dy_cost = (cy1 - cy0) / (2.0 * costmap_->getResolution());

  Eigen::Vector2d g(dx_cost, dy_cost);
  if (g.norm() < 1e-6) return std::nullopt;
  return g; // kierunek wzrostu kosztu ⇒ „odpychamy” w +grad
}

void AutoFixingTrajectoryValidator::laplacianSmoothXY(
  Eigen::ArrayXXf & traj, const std::vector<bool> &locked, int iters) const
{
  const int N = traj.rows();
  if (N < 3 || iters <= 0) return;
  for (int it = 0; it < iters; ++it) {
    for (int i = 1; i < N - 1; ++i) {
      if (locked[i]) continue;
      const Eigen::Vector2f prev = traj.row(i - 1).segment<2>(0);
      const Eigen::Vector2f curr = traj.row(i).segment<2>(0);
      const Eigen::Vector2f next = traj.row(i + 1).segment<2>(0);
      const Eigen::Vector2f lap = (prev - 2.f * curr + next);
      traj(i, 0) += 0.25f * lap.x();
      traj(i, 1) += 0.25f * lap.y();
    }
  }
}

void AutoFixingTrajectoryValidator::smoothYaw(Eigen::ArrayXXf & traj) const
{
  const int N = traj.rows();
  if (N < 3) return;
  for (int i = 1; i < N - 1; ++i) {
    const float t_prev = traj(i - 1, 2), t_cur = traj(i, 2), t_next = traj(i + 1, 2);
    traj(i, 2) = 0.25f * t_prev + 0.5f * t_cur + 0.25f * t_next;
  }
}

bool AutoFixingTrajectoryValidator::tryRepair(Eigen::ArrayXXf & traj, int start_idx, int end_idx) const
{
  const int Ncap = static_cast<int>(std::min<unsigned int>(traj.rows(), traj_samples_to_evaluate_));
  const int s = std::clamp(start_idx, 0, Ncap);
  const int e = std::clamp(end_idx,   0, Ncap);
  if (e - s <= 0) return true;

  std::vector<double> accum(e - s, 0.0);
  std::vector<bool>   locked(e - s, false);

  auto safe_idx = [&](int i) { return isSafe(traj(i, 0), traj(i, 1), traj(i, 2)); };

  for (int i = s; i < e; ++i) {
    locked[i - s] = safe_idx(i);
  }

  for (int it = 0; it < max_iters_; ++it) {
    bool improved = false;

    for (int i = s; i < e; ++i) {
      if (locked[i - s]) continue;

      const double x = traj(i, 0), y = traj(i, 1), th = traj(i, 2);
      double clr = clearanceAt(x, y);

      // kierunek: gradient kosztu lub fallback
      Eigen::Vector2d dir(0.0, 0.0);
      if (auto g = gradientAt(x, y)) {
        dir = -(*g);
      } else {
        const int K = 16;
        double best_gain = -1e9;
        for (int k = 0; k < K; ++k) {
          const double ang = 2.0 * M_PI * k / K;
          const Eigen::Vector2d cand(std::cos(ang), std::sin(ang));
          const double nx = x + 0.02 * cand.x();
          const double ny = y + 0.02 * cand.y();
          const double gain = clearanceAt(nx, ny) - clr;
          if (gain > best_gain) { best_gain = gain; dir = cand; }
        }
      }

      const double nrm = dir.norm();
      if (nrm < 1e-6) continue;
      dir /= nrm;

        const bool in_collision = isCollisionAt(x, y, th);
        double step = 0.0;
        if (in_collision) {
        // Silniejszy krok startowy, żeby wyjść poza INSCRIBED
        const double min_step = std::max({ 0.5 * costmap_->getResolution(),
                                            0.25 * safe_clearance_,
                                            safe_clearance_ + 0.5 * costmap_->getResolution() });
        step = std::min(std::max(min_step, step_gain_ * (safe_clearance_ + costmap_->getResolution())),
                        max_step_);
        } else {
        clr = clearanceAt(x, y);
        const double deficit = std::max(0.0, safe_clearance_ - clr);
        step = std::min(max_step_, step_gain_ * deficit);
        }
      double room = std::max(0.0, max_total_disp_ - accum[i - s]);
        if (room <= 1e-6) continue;
        if (step > room) step = room;


      // limit przemieszczenia per-punkt (względem zakresu [s,e))
      if (accum[i - s] + step > max_total_disp_) {
        step = std::max(0.0, max_total_disp_ - accum[i - s]);
      }
      if (step <= 1e-6) continue;

      double nx = x + step * dir.x();
      double ny = y + step * dir.y();

      // krótki line-search: nie wchodź w kolizję
      int ls = 0;
      while (ls < 4 && isCollisionAt(nx, ny, th)) {
        step *= 0.5; nx = x + step * dir.x(); ny = y + step * dir.y(); ++ls;
      }

      // jeśli footprint nadal „zbyt blisko”, wysuń delikatnie w górę
      if (consider_footprint_) {
        int ls_up = 0;
        while (ls_up < 4) {
          const double c = fp_checker_->footprintCostAtPose(nx, ny, th, costmap_ros_->getRobotFootprint());
          if (c < static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) break;
          double extra = 0.5 * step;

          // nie przekraczaj max_total_disp_
          room = std::max(0.0, max_total_disp_ - accum[i - s] - step);
          if (room <= 1e-6) break;
          if (extra > room) extra = room;

          nx += extra * dir.x();
          ny += extra * dir.y();
          step += extra;
          ++ls_up;
        }
      }

      if (step <= 1e-6) continue; // nic nie zrobiliśmy

      traj(i, 0) = static_cast<float>(nx);
      traj(i, 1) = static_cast<float>(ny);
      accum[i - s] += step;
      improved = true;
    }

    // smoothing z pełnym wektorem locków w przestrzeni [0..rows)
    std::vector<bool> locked_full(traj.rows(), false);
    for (int i = s; i < e; ++i) locked_full[i] = locked[i - s];
    laplacianSmoothXY(traj, locked_full, smooth_iters_);
    smoothYaw(traj);

    // zaktualizuj maskę bezpiecznych w zakresie
    bool all_safe = true;
    for (int i = s; i < e; ++i) {
      if (!safe_idx(i)) { all_safe = false; break; }
      locked[i - s] = true;
    }
    if (all_safe) return true;
    if (!improved) break;
  }

  // częściowo lub wcale się nie udało
  return false;
}

ValidationResult AutoFixingTrajectoryValidator::validateTrajectory(
  const Eigen::ArrayXXf & optimal_trajectory,
  const models::ControlSequence & control_sequence,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::Pose & goal)
{
  (void)control_sequence;
  (void)robot_pose;
  (void)robot_speed;
  (void)plan;
  (void)goal;

  auto node = node_.lock();
  RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
    "[%s] validateTrajectory called. samples=%u", name_.c_str(), traj_samples_to_evaluate_);

  has_fixed_ = false;
  fixed_traj_ = optimal_trajectory; // kopia robocza

  // 1) szybka weryfikacja
  int first_coll_idx = -1;
  const size_t limit = std::min<size_t>(traj_samples_to_evaluate_, fixed_traj_.rows());
  for (size_t i = 0; i < limit; ++i) {
    const double x = fixed_traj_(i, 0), y = fixed_traj_(i, 1), th = fixed_traj_(i, 2);
    if (!isSafe(x, y, th)) { first_coll_idx = static_cast<int>(i); break; }
  }
  
  if (first_coll_idx == 0) {
  const double old_disp = max_total_disp_;
  const int    old_iters = max_iters_;
  const double old_step  = max_step_;

  max_total_disp_ = std::max(max_total_disp_, 0.8);  // było 0.5
  max_iters_      = std::max(max_iters_, 14);        // było 10
  max_step_       = std::max(max_step_, 0.08);       // ≥ r

  const int window_samples = std::max(1,
    static_cast<int>(std::ceil(auto_fix_ttc_window_ / std::max(1e-6f, model_dt_))));
  const bool ok0 = tryRepair(fixed_traj_, 0, std::min<int>(limit, window_samples));

  // Przywróć
  max_total_disp_ = old_disp;
  max_iters_ = old_iters;
  max_step_  = old_step;

  if (!ok0) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "[%s] Collision at sample 0: requesting soft reset for re-sampling.",
      name_.c_str());
    return ValidationResult::SOFT_RESET;
  }

  // Drugi pass: potwierdź brak kolizji po naprawie TTC=0 na całym oknie
  bool clean = true;
  for (size_t j = 0; j < limit; ++j) {
    if (!isSafe(fixed_traj_(j,0), fixed_traj_(j,1), fixed_traj_(j,2))) { clean = false; break; }
  }
  if (clean) {
    has_fixed_ = true;
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "[%s] Auto-fix SUCCESS at TTC=0: using repaired trajectory.", name_.c_str());
    return ValidationResult::SUCCESS;  // ważne: NIE resetuj, pozwól kontrolerowi użyć fixed_traj_
  }

  // Inaczej wpadnij do standardowej ścieżki naprawy dalej w kodzie
}



  if (first_coll_idx > 0) {
    const double ttc = static_cast<double>(first_coll_idx) * static_cast<double>(model_dt_);
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "[%s] Collision predicted in %.2f s at sample %d (%.2f, %.2f). Attempting auto-fix...",
      name_.c_str(), ttc, first_coll_idx, fixed_traj_(first_coll_idx, 0), fixed_traj_(first_coll_idx, 1));

    // zakres naprawy: [first - pre, first + window]
    const int window_samples = std::max(1, static_cast<int>(std::ceil(
      auto_fix_ttc_window_ / std::max(1e-6f, model_dt_))));
    const int pre       = std::min(6, first_coll_idx); // ~ 6 próbek wstecz
    const int start_idx = first_coll_idx - pre;
    const int end_idx   = std::min<int>(static_cast<int>(limit), first_coll_idx + window_samples);

    // 2) próba naprawy
    const bool ok = tryRepair(fixed_traj_, start_idx, end_idx);
    if (!ok) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "[%s] Auto-fix FAILED: trajectory still in collision after repair attempts.",
        name_.c_str());
      return ValidationResult::SOFT_RESET;
    }

    // 3) potwierdź brak kolizji po naprawie
    for (size_t j = 0; j < limit; ++j) {
      if (!isSafe(fixed_traj_(j, 0), fixed_traj_(j, 1), fixed_traj_(j, 2))) {
        RCLCPP_ERROR_THROTTLE(
          node->get_logger(), *node->get_clock(), 1000,
          "[%s] Auto-fix incomplete: remaining collision at sample %zu.",
          name_.c_str(), j);
        return ValidationResult::SOFT_RESET;
      }
    }

    has_fixed_ = true;
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "[%s] Auto-fix SUCCESS: repaired up to %.2f s window; TTC was %.2f s.",
      name_.c_str(), (first_coll_idx + window_samples) * model_dt_, ttc);

    return ValidationResult::SUCCESS;
  }

  // już było dobrze
  return ValidationResult::SUCCESS;
}

} // namespace mppi

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi::AutoFixingTrajectoryValidator, mppi::OptimalTrajectoryValidator)
