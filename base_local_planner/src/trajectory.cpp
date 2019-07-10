/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <base_local_planner/trajectory.h>

namespace base_local_planner {
  Trajectory::Trajectory()
    : xv_(0.0), yv_(0.0), thetav_(0.0), cost_(-1.0)
  {
  }

  Trajectory::Trajectory(double xv, double yv, double thetav, double time_delta, unsigned int num_pts)
    : xv_(xv), yv_(yv), thetav_(thetav), cost_(-1.0), time_delta_(time_delta), x_pts_(num_pts), y_pts_(num_pts), th_pts_(num_pts)
  {
  }

  void Trajectory::getPoint(unsigned int index, double& x, double& y, double& th) const {
    x = x_pts_[index];
    y = y_pts_[index];
    th = th_pts_[index];
  }

  void Trajectory::setPoint(unsigned int index, double x, double y, double th){
    x_pts_[index] = x;
    y_pts_[index] = y;
    th_pts_[index] = th;
  }

  void Trajectory::addPoint(double x, double y, double th){
    x_pts_.push_back(x);
    y_pts_.push_back(y);
    th_pts_.push_back(th);
  }

  void Trajectory::resetPoints(){
    x_pts_.clear();
    y_pts_.clear();
    th_pts_.clear();
  }

  void Trajectory::getEndpoint(double& x, double& y, double& th) const {
    x = x_pts_.back();
    y = y_pts_.back();
    th = th_pts_.back();
  }

  unsigned int Trajectory::getPointsSize() const {
    return x_pts_.size();
  }
};
