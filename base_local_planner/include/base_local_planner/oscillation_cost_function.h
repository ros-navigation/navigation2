/*********************************************************************
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: TKruse
 *********************************************************************/

#ifndef OSCILLATION_COST_FUNCTION_H_
#define OSCILLATION_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <Eigen/Core>

namespace base_local_planner {

class OscillationCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  OscillationCostFunction();
  virtual ~OscillationCostFunction();

  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};

  /**
   * @brief  Reset the oscillation flags for the local planner
   */
  void resetOscillationFlags();


  void updateOscillationFlags(Eigen::Vector3f pos, base_local_planner::Trajectory* traj, double min_vel_trans);

  void setOscillationResetDist(double dist, double angle);

private:

  void resetOscillationFlagsIfPossible(const Eigen::Vector3f& pos, const Eigen::Vector3f& prev);

  /**
   * @brief  Given a trajectory that's selected, set flags if needed to
   * prevent the robot from oscillating
   * @param  t The selected trajectory
   * @return True if a flag was set, false otherwise
   */
  bool setOscillationFlags(base_local_planner::Trajectory* t, double min_vel_trans);

  // flags
  bool strafe_pos_only_, strafe_neg_only_, strafing_pos_, strafing_neg_;
  bool rot_pos_only_, rot_neg_only_, rotating_pos_, rotating_neg_;
  bool forward_pos_only_, forward_neg_only_, forward_pos_, forward_neg_;

  // param
  double oscillation_reset_dist_, oscillation_reset_angle_;

  Eigen::Vector3f prev_stationary_pos_;
};

} /* namespace base_local_planner */
#endif /* OSCILLATION_COST_FUNCTION_H_ */
