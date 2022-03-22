#ifndef MPPIC__OPTIMIZATION__MOTION_MODELS_HPP_
#define MPPIC__OPTIMIZATION__MOTION_MODELS_HPP_

#include <cstdint>

#include "mppic/tensor_wrappers/state.hpp"

namespace mppi
{

class MotionModel
{
public:
  MotionModel() = default;
  ~MotionModel() = default;

  /**
   * @brief Predict velocities for given trajectories the next time step
   *
   * @param state for given time_step, tensor of shape
   * [batch_size, ...] where last dim could be 5 or 7 depending on motion model used
   *
   * @return predicted velocities of the robot: tensor of shape [batch_size, ... ]
   * where last dim could be 2 or 3 depending on motion model used
   */
  virtual xt::xtensor<double, 2> predict(
    const xt::xtensor<double, 2> & state, const optimization::StateIdxes & idx)
  {
    return xt::view(state, xt::all(), xt::range(idx.cbegin(), idx.cend()));
  }

  virtual bool isHolonomic() const = 0;
};

class DiffDriveMotionModel : public MotionModel
{
public:
  virtual bool isHolonomic() const override {return false;}
};

class OmniMotionModel : public MotionModel
{
public:
  virtual bool isHolonomic() const override {return true;}
};

class AckermannMotionModel : public MotionModel
{
public:
  virtual xt::xtensor<double, 2> predict(
    const xt::xtensor<double, 2> & /*state*/, const optimization::StateIdxes & /*idx*/) override
  {
    throw std::runtime_error("Ackermann motion model not yet implemented");
  }

  virtual bool isHolonomic() const override {return false;}
};

} // namespace mppi

#endif  // MPPIC__MOTION_MODELS_HPP_
