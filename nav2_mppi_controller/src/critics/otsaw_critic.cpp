// Copyright (c) 2024 OTSAW

#include "nav2_mppi_controller/critics/otsaw_critic.hpp"

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
}

void OtsawCritic::score(CriticData & data)
{
  data.costs += test_gpu_fn();
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::OtsawCritic, mppi::critics::CriticFunction)
