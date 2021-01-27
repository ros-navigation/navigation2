// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
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

#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
class DiffDriveOdomMotionModel : public SampleMotionModel
{
public:
  geometry_msgs::msg::TransformStamped getMostLikelyPose(
    const geometry_msgs::msg::TransformStamped & prev_odom,
    const geometry_msgs::msg::TransformStamped & curr_odom,
    const geometry_msgs::msg::TransformStamped & prev_pose) override;

  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

private:
  // Noise parameters
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PLUGINS__SAMPLE_MOTION_MODELS__DIFF_DRIVE_ODOM_MOTION_MODEL_HPP_
