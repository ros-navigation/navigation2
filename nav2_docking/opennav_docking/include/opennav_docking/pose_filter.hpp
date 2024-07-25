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

#ifndef OPENNAV_DOCKING__POSE_FILTER_HPP_
#define OPENNAV_DOCKING__POSE_FILTER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace opennav_docking
{

/**
 * @class opennav_docking::PoseFilter
 * @brief Filter for a sequence of pose measurements.
 */
class PoseFilter
{
public:
  /**
   * @brief Create a pose filter instance.
   * @param coef Filtering coefficient. Valid range is 0-1, where 0 means take the new measurement
   * @param timeout If time between measurments exceeds this value, take the new measurement.
   */
  PoseFilter(double coef, double timeout);

  /**
   * @brief Update the filter.
   * @param measurement The new pose measurement.
   * @returns Filtered measurement
   */
  geometry_msgs::msg::PoseStamped update(const geometry_msgs::msg::PoseStamped & measurement);

protected:
  void filter(double & filt, double meas);

  double coef_, timeout_;
  geometry_msgs::msg::PoseStamped pose_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__POSE_FILTER_HPP_
