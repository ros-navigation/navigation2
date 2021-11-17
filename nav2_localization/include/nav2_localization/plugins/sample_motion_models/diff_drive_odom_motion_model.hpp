// Copyright (c) 2021 Khaled SAAD, Jose M. TORRES-CAMARA and Marwan TAHER
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef NAV2_LOCALIZATION__PLUGINS__SAMPLE_MOTION_MODELS__DIFF_DRIVE_ODOM_MOTION_MODEL_HPP_
#define NAV2_LOCALIZATION__PLUGINS__SAMPLE_MOTION_MODELS__DIFF_DRIVE_ODOM_MOTION_MODEL_HPP_

#include <random>
#include <memory>

#include "nav2_localization/plugins/sample_motion_models/sample_motion_model_base.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
class DiffDriveOdomMotionModel : public SampleMotionModel
{
public:
  geometry_msgs::msg::Pose getMostLikelyPose(
    const nav_msgs::msg::Odometry & prev_odom,
    const nav_msgs::msg::Odometry & curr_odom,
    const geometry_msgs::msg::Pose & prev_pose) override;

  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

protected:
  struct MotionComponents
  {
    MotionComponents(const double & rot_1, const double & trans, const double & rot_2)
    : rot_1_(rot_1), trans_(trans), rot_2_(rot_2) {}
    double rot_1_, trans_, rot_2_;
  };

  MotionComponents calculateIdealMotionComponents(
    const geometry_msgs::msg::Pose & prev,
    const geometry_msgs::msg::Pose & curr
  );
  double calculateNoisyRot(const double & rot, const double & trans);
  double calculateNoisyTrans(const double & rot1, const double & trans, const double & rot2);

  // Noise parameters
  double rot_rot_noise_parm_;
  double trans_rot_noise_parm_;
  double trans_trans_noise_parm_;
  double rot_trans_noise_param_;

  std::shared_ptr<std::mt19937> rand_num_gen_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PLUGINS__SAMPLE_MOTION_MODELS__DIFF_DRIVE_ODOM_MOTION_MODEL_HPP_
