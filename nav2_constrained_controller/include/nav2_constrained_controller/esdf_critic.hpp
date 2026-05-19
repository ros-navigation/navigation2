// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// EsdfCritic — MPPI critic scoring trajectories from the local ESDF.
//
// Namespace mppi::critics is required: CriticManager prepends "mppi::critics::"
// to every critic name from the YAML, so this class must live there for pluginlib
// to resolve "mppi::critics::EsdfCritic" correctly.

#ifndef NAV2_CONSTRAINED_CONTROLLER__ESDF_CRITIC_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__ESDF_CRITIC_HPP_

#include <array>
#include <string>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/critic_data.hpp"

#include "nav2_constrained_controller/esdf_registry.hpp"

namespace mppi::critics
{

class EsdfCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  std::string controller_name_;

  float Lext_{0.50f};      // footprint half-length + margin
  float db_{0.375f};       // footprint half-width
  float d_safe_{0.02f};    // safety margin (metres)

  float cost_weight_{100.0f};
  float collision_cost_{10000.0f};
  float near_dist_{0.06f};   // gradient starts at 6cm — creates real cost signal at corridor clearances
  float sigma_{0.03f};       // gradient spans the 6cm danger zone

  unsigned int step_stride_{2};
};

}  // namespace mppi::critics

#endif  // NAV2_CONSTRAINED_CONTROLLER__ESDF_CRITIC_HPP_
