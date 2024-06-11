// Copyright (c) 2024 Open Navigation LLC
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
// limitations under the License.

#include "opennav_docking/pose_filter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace opennav_docking
{

PoseFilter::PoseFilter(double coef, double timeout)
{
  coef_ = coef;
  timeout_ = timeout;
  pose_.header.stamp = rclcpp::Time(0);
}

geometry_msgs::msg::PoseStamped
PoseFilter::update(const geometry_msgs::msg::PoseStamped & measurement)
{
  if (coef_ <= 0.0) {
    // No filtering
    return measurement;
  }

  if ((rclcpp::Time(measurement.header.stamp) - pose_.header.stamp).seconds() > timeout_) {
    pose_ = measurement;
  } else if (pose_.header.frame_id != measurement.header.frame_id) {
    pose_ = measurement;
  } else {
    // Copy header
    pose_.header = measurement.header;

    // Filter position
    filter(pose_.pose.position.x, measurement.pose.position.x);
    filter(pose_.pose.position.y, measurement.pose.position.y);
    filter(pose_.pose.position.z, measurement.pose.position.z);

    // Filter orientation
    tf2::Quaternion f_quat, m_quat;
    tf2::fromMsg(measurement.pose.orientation, m_quat);
    tf2::fromMsg(pose_.pose.orientation, f_quat);
    f_quat = f_quat.slerp(m_quat, coef_);
    pose_.pose.orientation = tf2::toMsg(f_quat);
  }

  return pose_;
}

void PoseFilter::filter(double & filt, double meas)
{
  filt = (1 - coef_) * filt + coef_ * meas;
}

}  // namespace opennav_docking
