/***********************************************************
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
 ***********************************************************/


#ifndef __base_local_planner__LOCALPLANNERLIMITS_H__
#define __base_local_planner__LOCALPLANNERLIMITS_H__

#include <Eigen/Core>

namespace base_local_planner
{
class LocalPlannerLimits
{
public:

  double max_vel_trans;
  double min_vel_trans;
  double max_vel_x;
  double min_vel_x;
  double max_vel_y;
  double min_vel_y;
  double max_vel_theta;
  double min_vel_theta;
  double acc_lim_x;
  double acc_lim_y;
  double acc_lim_theta;
  double acc_lim_trans;
  bool   prune_plan;
  double xy_goal_tolerance;
  double yaw_goal_tolerance;
  double trans_stopped_vel;
  double theta_stopped_vel;
  bool   restore_defaults;

  LocalPlannerLimits() {}

  LocalPlannerLimits(
      double nmax_vel_trans,
      double nmin_vel_trans,
      double nmax_vel_x,
      double nmin_vel_x,
      double nmax_vel_y,
      double nmin_vel_y,
      double nmax_vel_theta,
      double nmin_vel_theta,
      double nacc_lim_x,
      double nacc_lim_y,
      double nacc_lim_theta,
      double nacc_lim_trans,
      double nxy_goal_tolerance,
      double nyaw_goal_tolerance,
      bool   nprune_plan = true,
      double ntrans_stopped_vel = 0.1,
      double ntheta_stopped_vel = 0.1):
        max_vel_trans(nmax_vel_trans),
        min_vel_trans(nmin_vel_trans),
        max_vel_x(nmax_vel_x),
        min_vel_x(nmin_vel_x),
        max_vel_y(nmax_vel_y),
        min_vel_y(nmin_vel_y),
        max_vel_theta(nmax_vel_theta),
        min_vel_theta(nmin_vel_theta),
        acc_lim_x(nacc_lim_x),
        acc_lim_y(nacc_lim_y),
        acc_lim_theta(nacc_lim_theta),
        acc_lim_trans(nacc_lim_trans),
        prune_plan(nprune_plan),
        xy_goal_tolerance(nxy_goal_tolerance),
        yaw_goal_tolerance(nyaw_goal_tolerance),
        trans_stopped_vel(ntrans_stopped_vel),
        theta_stopped_vel(ntheta_stopped_vel) {}

  ~LocalPlannerLimits() {}

  /**
   * @brief  Get the acceleration limits of the robot
   * @return  The acceleration limits of the robot
   */
  Eigen::Vector3f getAccLimits() {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[1] = acc_lim_y;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

};

}
#endif // __LOCALPLANNERLIMITS_H__
