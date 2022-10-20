/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_
#define DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_

#include <algorithm>
#include <cmath>

namespace dwb_plugins
{

const double EPSILON = 1E-5;

/**
 * @brief Given initial conditions and a time, figure out the end velocity
 *
 * @param v0 Initial velocity
 * @param accel The acceleration rate
 * @param decel The decceleration rate
 * @param dt Delta time - amount of time to project into the future
 * @param target target velocity
 * @return The velocity dt seconds after v0.
 */
inline double projectVelocity(double v0, double accel, double decel, double dt, double target)
{
  double v1;
  if (v0 < target) {
    v1 = v0 + accel * dt;
    return std::min(target, v1);
  } else {
    v1 = v0 + decel * dt;
    return std::max(target, v1);
  }
}

/**
 * @class OneDVelocityIterator
 * @brief An iterator for generating a number of samples in a range
 *
 * In its simplest usage, this gives us N (num_samples) different velocities that are reachable
 * given our current velocity. However, there is some fancy logic around zero velocities and
 * the min/max velocities
 *
 * If the current velocity is 2 m/s, and the acceleration limit is 1 m/ss and the acc_time is 1 s,
 * this class would provide velocities between 1 m/s and 3 m/s.
 *
 *
 *
 */
class OneDVelocityIterator
{
public:
  /**
   * @brief Constructor for the velocity iterator
   *
   * @param current Current velocity
   * @param min Minimum velocity allowable
   * @param max Maximum velocity allowable
   * @param acc_limit Acceleration Limit
   * @param decel_limit Deceleration Limit
   * @param num_samples The number of samples to return
   */
  OneDVelocityIterator(
    double current, double min, double max, double acc_limit, double decel_limit, double acc_time,
    int num_samples)
  {
    if (current < min) {
      current = min;
    } else if (current > max) {
      current = max;
    }
    max_vel_ = projectVelocity(current, acc_limit, decel_limit, acc_time, max);
    min_vel_ = projectVelocity(current, acc_limit, decel_limit, acc_time, min);
    reset();

    if (fabs(min_vel_ - max_vel_) < EPSILON) {
      increment_ = 1.0;
      return;
    }
    num_samples = std::max(2, num_samples);

    // e.g. for 4 samples, split distance in 3 even parts
    increment_ = (max_vel_ - min_vel_) / std::max(1, (num_samples - 1));
  }

  /**
   * @brief Get the next velocity available
   */
  double getVelocity() const
  {
    if (return_zero_now_) {return 0.0;}
    return current_;
  }

  /**
   * @brief Increment the iterator
   */
  OneDVelocityIterator & operator++()
  {
    if (return_zero_ && current_ < 0.0 && current_ + increment_ > 0.0 &&
      current_ + increment_ <= max_vel_ + EPSILON)
    {
      return_zero_now_ = true;
      return_zero_ = false;
    } else {
      current_ += increment_;
      return_zero_now_ = false;
    }
    return *this;
  }

  /**
   * @brief Reset back to the first velocity
   */
  void reset()
  {
    current_ = min_vel_;
    return_zero_ = true;
    return_zero_now_ = false;
  }

  /**
   * If we have returned all the velocities for this iteration
   */
  bool isFinished() const
  {
    return current_ > max_vel_ + EPSILON;
  }

private:
  bool return_zero_, return_zero_now_;
  double min_vel_, max_vel_;
  double current_;
  double increment_;
};
}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__ONE_D_VELOCITY_ITERATOR_HPP_
