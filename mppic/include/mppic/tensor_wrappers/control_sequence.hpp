#ifndef MPPIC__OPTIMIZATION__TENSOR_WRAPPERS__CONTROL_SEQUENCE_HPP_
#define MPPIC__OPTIMIZATION__TENSOR_WRAPPERS__CONTROL_SEQUENCE_HPP_

#include <array>
#include <cstdint>

#include <xtensor/xtensor.hpp>

#include "mppic/motion_models.hpp"

namespace mppi::optimization
{

/**
 * @brief Keeps named indexes of control sequence last dimension variables
 */
class ControlSequnceIdxes
{
public:
  unsigned int dim() const {return dim_;}

  uint8_t vx() const {return vx_;}
  uint8_t vy() const {return vy_;}
  uint8_t wz() const {return wz_;}

  void setLayout(const bool is_holonomic)
  {
    // Layout changes to include "Y" components if holonomic
    if (is_holonomic) {
      vx_ = 0;
      vy_ = 1;
      wz_ = 2;
      dim_ = 3;
    } else {
      vx_ = 0;
      wz_ = 1;
      dim_ = 2;
    }
  }

private:
  uint8_t vx_{0};
  uint8_t vy_{0};
  uint8_t wz_{0};
  unsigned int dim_{0};
};

/**
 * @brief Contains trajectory controls (in data) for each time step (vx, wz, [vy])
 * last dimension layout described by ControlSequnceIdxes
 */
struct ControlSequence
{
  xt::xtensor<double, 2> data;
  ControlSequnceIdxes idx;

  void reset(unsigned int time_steps) {data = xt::zeros<double>({time_steps, idx.dim()});}
};

} // namespace mppi::optimization

#endif  // MPPIC__OPTIMIZATION__TENSOR_WRAPPERS__CONTROL_SEQUENCE_HPP_
