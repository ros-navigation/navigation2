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

#ifndef NAV2_PATH_CLASSIFIER__CLASSIFICATION_FILTER_HPP_
#define NAV2_PATH_CLASSIFIER__CLASSIFICATION_FILTER_HPP_

#include <vector>

#include "nav2_msgs/msg/classified_pose.hpp"
#include "rclcpp/logger.hpp"

namespace nav2_path_classifier
{

/**
 * @class ClassificationFilter
 * @brief Hysteresis filter for the per-pose classes of a classified path.
 */
class ClassificationFilter
{
public:
  /**
   * @brief A constructor for nav2_path_classifier::ClassificationFilter
   * @param hysteresis_window Consecutive poses of a new class needed to confirm a transition
   * @param logger Logger used to report invalid parameters
   */
  ClassificationFilter(int hysteresis_window, const rclcpp::Logger & logger);

  /**
   * @brief A destructor for nav2_path_classifier::ClassificationFilter
   */
  ~ClassificationFilter() = default;

  /**
   * @brief Filter the classes of a classified path in place.
   * @param classified_path Classified poses in path order, class_type is overwritten with the
   *        filtered class
   */
  void filter(std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path) const;

protected:
  int hysteresis_window_;
  rclcpp::Logger logger_;
};

}  // namespace nav2_path_classifier

#endif  // NAV2_PATH_CLASSIFIER__CLASSIFICATION_FILTER_HPP_
