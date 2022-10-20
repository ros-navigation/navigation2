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

#ifndef DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_
#define DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{

/**
 * @struct KinematicParameters
 * @brief A struct containing one representation of the robot's kinematics
 */
struct KinematicParameters
{
  friend class KinematicsHandler;

  inline double getMinX() {return min_vel_x_;}
  inline double getMaxX() {return max_vel_x_;}
  inline double getAccX() {return acc_lim_x_;}
  inline double getDecelX() {return decel_lim_x_;}

  inline double getMinY() {return min_vel_y_;}
  inline double getMaxY() {return max_vel_y_;}
  inline double getAccY() {return acc_lim_y_;}
  inline double getDecelY() {return decel_lim_y_;}

  inline double getMinSpeedXY() {return min_speed_xy_;}
  inline double getMaxSpeedXY() {return max_speed_xy_;}

  inline double getMinTheta() {return -max_vel_theta_;}
  inline double getMaxTheta() {return max_vel_theta_;}
  inline double getAccTheta() {return acc_lim_theta_;}
  inline double getDecelTheta() {return decel_lim_theta_;}
  inline double getMinSpeedTheta() {return min_speed_theta_;}

  inline double getMinSpeedXY_SQ() {return min_speed_xy_sq_;}
  inline double getMaxSpeedXY_SQ() {return max_speed_xy_sq_;}

protected:
  // For parameter descriptions, see cfg/KinematicParams.cfg
  double min_vel_x_{0};
  double min_vel_y_{0};
  double max_vel_x_{0};
  double max_vel_y_{0};
  double base_max_vel_x_{0};
  double base_max_vel_y_{0};
  double max_vel_theta_{0};
  double base_max_vel_theta_{0};
  double min_speed_xy_{0};
  double max_speed_xy_{0};
  double base_max_speed_xy_{0};
  double min_speed_theta_{0};
  double acc_lim_x_{0};
  double acc_lim_y_{0};
  double acc_lim_theta_{0};
  double decel_lim_x_{0};
  double decel_lim_y_{0};
  double decel_lim_theta_{0};

  // Cached square values of min_speed_xy and max_speed_xy
  double min_speed_xy_sq_{0};
  double max_speed_xy_sq_{0};
};

/**
 * @class KinematicsHandler
 * @brief A class managing the representation of the robot's kinematics
 */
class KinematicsHandler
{
public:
  KinematicsHandler();
  ~KinematicsHandler();
  void initialize(const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & plugin_name);

  inline KinematicParameters getKinematics() {return *kinematics_.load();}

  void setSpeedLimit(const double & speed_limit, const bool & percentage);

  using Ptr = std::shared_ptr<KinematicsHandler>;

protected:
  std::atomic<KinematicParameters *> kinematics_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  void update_kinematics(KinematicParameters kinematics);
  std::string plugin_name_;
};

}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__KINEMATIC_PARAMETERS_HPP_
