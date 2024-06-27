// Copyright (c) 2024 OTSAW

#include "nav2_mppi_controller/critics/otsaw_critic.hpp"
#include <xtensor/xio.hpp>

namespace mppi::critics
{

void OtsawCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);
  RCLCPP_INFO(
    logger_, "OtsawCritic instantiated with %d power and %f weight.",
    power_, weight_);

  float vx_max, vy_max, vx_min;
  getParentParam(vx_max, "vx_max", 0.5);
  getParentParam(vy_max, "vy_max", 0.0);
  getParentParam(vx_min, "vx_min", -0.35);

  const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
}

void OtsawCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;
  }

  // Create thrust device vectors and copy data from xtensor arrays
  // std::cout << "vx(" << xt::adapt(data.state.vx.shape()) << "): "<< data.state.vx << std::endl;

  std::vector<float> vx(data.state.vx.begin(), data.state.vx.end());
  std::vector<float> vy(data.state.vy.begin(), data.state.vy.end());
  std::vector<float> costs = calc_constraint_critics_cost(
    vx,
    vy,
    max_vel_,
    min_vel_,
    data.model_dt
  );

  xt::xarray<float> v_costs = xt::adapt(
    costs.data(),
    costs.size(),
    xt::no_ownership(),
    data.state.vx.shape()
  );

  data.costs += xt::pow(
    xt::sum(v_costs, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::OtsawCritic, mppi::critics::CriticFunction)
