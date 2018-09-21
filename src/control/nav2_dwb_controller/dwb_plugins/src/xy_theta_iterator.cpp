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

#include <dwb_plugins/xy_theta_iterator.h>
#include <nav_2d_utils/parameters.h>

namespace dwb_plugins
{
void XYThetaIterator::initialize(ros::NodeHandle& nh, KinematicParameters::Ptr kinematics)
{
  kinematics_ = kinematics;
  nh.param("vx_samples", vx_samples_, 20);
  nh.param("vy_samples", vy_samples_, 5);
  vtheta_samples_ = nav_2d_utils::loadParameterWithDeprecation(nh, "vtheta_samples", "vth_samples", 20);
}

void XYThetaIterator::startNewIteration(const nav_2d_msgs::Twist2D& current_velocity, double dt)
{
  x_it_ = std::make_shared<OneDVelocityIterator>(current_velocity.x, kinematics_->getMinX(), kinematics_->getMaxX(),
                                                 kinematics_->getAccX(), kinematics_->getDecelX(), dt, vx_samples_);
  y_it_ = std::make_shared<OneDVelocityIterator>(current_velocity.y, kinematics_->getMinY(), kinematics_->getMaxY(),
                                                 kinematics_->getAccY(), kinematics_->getDecelY(), dt, vy_samples_);
  th_it_ = std::make_shared<OneDVelocityIterator>(current_velocity.theta,
                                                  kinematics_->getMinTheta(), kinematics_->getMaxTheta(),
                                                  kinematics_->getAccTheta(), kinematics_->getDecelTheta(),
                                                  dt, vtheta_samples_);
  if (!isValidVelocity())
  {
    iterateToValidVelocity();
  }
}

bool XYThetaIterator::isValidVelocity()
{
  return kinematics_->isValidSpeed(x_it_->getVelocity(), y_it_->getVelocity(), th_it_->getVelocity());
}

bool XYThetaIterator::hasMoreTwists()
{
  return x_it_ && !x_it_->isFinished();
}


nav_2d_msgs::Twist2D XYThetaIterator::nextTwist()
{
  nav_2d_msgs::Twist2D velocity;
  velocity.x = x_it_->getVelocity();
  velocity.y = y_it_->getVelocity();
  velocity.theta = th_it_->getVelocity();

  iterateToValidVelocity();

  return velocity;
}

void XYThetaIterator::iterateToValidVelocity()
{
  bool valid = false;
  while (!valid && hasMoreTwists())
  {
    ++(*th_it_);
    if (th_it_->isFinished())
    {
      th_it_->reset();
      ++(*y_it_);
      if (y_it_->isFinished())
      {
        y_it_->reset();
        ++(*x_it_);
      }
    }
    valid = isValidVelocity();
  }
}

}  // namespace dwb_plugins
