// Copyright (c) 2024 Nav2 Contributors
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

#ifndef NAV2_DSTAR_LITE_PLANNER__PARAMETER_HANDLER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_dstar_lite_planner
{

struct Parameters
{
  bool allow_unknown{true};
  int max_iterations{100000};
  int replan_interval{10};
  double hysteresis_factor{1.05};
  int terminal_checking_interval{5000};
  bool use_final_approach_orientation{false};
};

class ParameterHandler
{
public:
  ParameterHandler(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::string & plugin_name,
    const rclcpp::Logger & logger);

  ~ParameterHandler();

  Parameters * getParams() {return &params_;}

  void activate();
  void deactivate();

  std::mutex & getMutex() {return mutex_;}

protected:
  Parameters params_;
  rclcpp::Logger logger_;
  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;
  std::mutex mutex_;

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__PARAMETER_HANDLER_HPP_
