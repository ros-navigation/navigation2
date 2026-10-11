// Copyright (c) 2026 Origin
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

#include "nav2_path_classifier/classification_filter.hpp"

#include <cstddef>
#include <cstdint>

#include "rclcpp/logging.hpp"

namespace nav2_path_classifier
{

ClassificationFilter::ClassificationFilter(
  int hysteresis_window, const rclcpp::Logger & logger)
: hysteresis_window_(hysteresis_window),
  logger_(logger)
{
  if (hysteresis_window_ < 1) {
    RCLCPP_WARN(logger_, "ClassificationFilter: hysteresis_window must be >= 1, clamping to 1");
    hysteresis_window_ = 1;
  }
}

void ClassificationFilter::filter(
  std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path) const
{
  if (classified_path.empty()) {
    return;
  }

  // No class is accepted until one is confirmed, so poses before it are treated as noise
  bool has_current = false;
  uint16_t current_class = 0;
  uint16_t pending_class = classified_path[0].class_type;
  int run_count = 0;

  for (size_t i = 0; i < classified_path.size(); ++i) {
    const uint16_t raw = classified_path[i].class_type;
    if (has_current && raw == current_class) {
      // Matches the accepted class, so drop any pending transition
      run_count = 0;
      pending_class = current_class;
    } else if (raw == pending_class) {
      ++run_count;
    } else {
      // A different class appeared, so start a new pending run
      pending_class = raw;
      run_count = 1;
    }

    if (run_count >= hysteresis_window_) {
      // Class confirmed. Backfill the pending run with it, or every pose so far
      // when it is the first confirmed class
      const size_t backfill_start =
        has_current ? i + 1 - static_cast<size_t>(run_count) : 0;
      current_class = pending_class;
      has_current = true;
      for (size_t j = backfill_start; j < i; ++j) {
        classified_path[j].class_type = current_class;
      }
      run_count = 0;
    }

    if (has_current) {
      classified_path[i].class_type = current_class;
    }
  }

  // No class was ever confirmed, so keep the first pose's class
  if (!has_current) {
    const uint16_t first_class = classified_path[0].class_type;
    for (auto & classified_pose : classified_path) {
      classified_pose.class_type = first_class;
    }
  }
}

}  // namespace nav2_path_classifier
