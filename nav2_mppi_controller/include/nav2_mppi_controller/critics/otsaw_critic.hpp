// Copyright (c) 2024 OTSAW

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OTSAW_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OTSAW_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/cuda/otsaw_critic_gpu.cuh"

namespace mppi::critics
{

/**
 * @class mppi::critics::OtsawCritic
 * @brief Critic objective function for otsaw product
 */
class OtsawCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OTSAW_CRITIC_HPP_
