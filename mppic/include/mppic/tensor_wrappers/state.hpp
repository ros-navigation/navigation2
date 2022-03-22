// Copyright 2022 FastSense, Samsung Research
#ifndef MPPIC__TENSOR_WRAPPERS__STATE_HPP_
#define MPPIC__TENSOR_WRAPPERS__STATE_HPP_

#include <array>
#include <cstdint>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xarray.hpp>

namespace mppi::optimization
{

/**
 * @brief Keeps named indexes of state last dimension variables
 */
class StateIdxes
{
public:
  uint8_t vbegin() const {return velocity_range_[0];}
  uint8_t vend() const {return velocity_range_[1];}
  uint8_t vx() const {return vx_;}
  uint8_t vy() const {return vy_;}
  uint8_t wz() const {return wz_;}

  uint8_t cbegin() const {return control_range_[0];}
  uint8_t cend() const {return control_range_[1];}
  uint8_t cvx() const {return cvx_;}
  uint8_t cvy() const {return cvy_;}
  uint8_t cwz() const {return cwz_;}

  uint8_t dt() const {return dt_;}
  unsigned int dim() const {return dim_;}

  void setLayout(const bool is_holonomic)
  {
    // Layout changes to include "Y" components if holonomic
    if (is_holonomic) {
      vx_ = 0;
      vy_ = 1;
      wz_ = 2;
      cvx_ = 3;
      cvy_ = 4;
      cwz_ = 5;
      dt_ = 6;
      dim_ = 7;
    } else {
      vx_ = 0;
      wz_ = 1;
      cvx_ = 2;
      cwz_ = 3;
      dt_ = 4;
      dim_ = 5;
    }

    velocity_range_[0] = vx_;
    velocity_range_[1] = cvx_;
    control_range_[0] = cvx_;
    control_range_[1] = dt_;
  }

private:
  uint8_t vx_{0};
  uint8_t vy_{0};
  uint8_t wz_{0};
  uint8_t cvx_{0};
  uint8_t cvy_{0};
  uint8_t cwz_{0};
  uint8_t dt_{0};
  std::array<uint8_t, 2> velocity_range_{0, 0};
  std::array<uint8_t, 2> control_range_{0, 0};

  unsigned int dim_{0};
};

/**
 * @brief State represent current the state of optimization problem.
 *
 * State stores state of the system for each trajectory. It has shape [ batch_size x time_steps x dim ].
 * Last dimension described by StateIdxes and consists of velocities, controls,
 * and amount of time between time steps (vx, [vy], wz, cvx, [cvy], cwz, dt)
 *
 **/
struct State
{
  xt::xtensor<double, 3> data;
  StateIdxes idx;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    data = xt::zeros<double>({batch_size, time_steps, idx.dim()});
  }

  auto getVelocitiesVX() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.vx());
  }

  auto getVelocitiesVX()
  {
    return xt::view(data, xt::all(), xt::all(), idx.vx());
  }

  auto getVelocitiesVY()
  {
    return xt::view(data, xt::all(), xt::all(), idx.vy());
  }

  auto getVelocitiesVY() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.vy());
  }

  auto getVelocitiesWZ() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.wz());
  }

  auto getVelocitiesWZ()
  {
    return xt::view(data, xt::all(), xt::all(), idx.wz());
  }

  auto getControlVelocitiesVX() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvx());
  }

  auto getControlVelocitiesVX()
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvx());
  }

  auto getControlVelocitiesVY()
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvy());
  }

  auto getControlVelocitiesVY() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.cvy());
  }

  auto getControlVelocitiesWZ() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.cwz());
  }

  auto getControlVelocitiesWZ()
  {
    return xt::view(data, xt::all(), xt::all(), idx.cwz());
  }

  auto getTimeIntervals()
  {
    return xt::view(data, xt::all(), xt::all(), idx.dt());
  }

  auto getTimeIntervals() const
  {
    return xt::view(data, xt::all(), xt::all(), idx.dt());
  }

  auto getControls() const
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.cbegin(), idx.cend()));
  }

  auto getControls()
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.cbegin(), idx.cend()));
  }

  auto getVelocities() const
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.vbegin(), idx.vend()));
  }

  auto getVelocities()
  {
    return xt::view(data, xt::all(), xt::all(), xt::range(idx.vbegin(), idx.vend()));
  }
};

}  // namespace mppi::optimization

#endif  // MPPIC__TENSOR_WRAPPERS__STATE_HPP_
