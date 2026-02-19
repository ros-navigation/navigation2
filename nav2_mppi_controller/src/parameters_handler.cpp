// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

ParametersHandler::ParametersHandler(
  const nav2::LifecycleNode::WeakPtr & parent, std::string & name)
{
  node_ = parent;
  auto node = node_.lock();
  node_name_ = node->get_name();
  logger_ = node->get_logger();
  name_ = name;
}

ParametersHandler::~ParametersHandler()
{
  auto node = node_.lock();
  if (post_set_param_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_param_handler_.get());
  }
  post_set_param_handler_.reset();
  if (on_set_param_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_param_handler_.get());
  }
  on_set_param_handler_.reset();
  if (pre_set_param_handler_ && node) {
    node->remove_pre_set_parameters_callback(pre_set_param_handler_.get());
  }
  pre_set_param_handler_.reset();
}

void ParametersHandler::start()
{
  auto node = node_.lock();

  auto get_param = getParamGetter(node_name_);
  get_param(verbose_, "verbose", false);
  post_set_param_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &ParametersHandler::updateParametersCallback, this,
      std::placeholders::_1));
  on_set_param_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParametersHandler::validateParameterUpdatesCallback, this,
      std::placeholders::_1));
  pre_set_param_handler_ = node->add_pre_set_parameters_callback(
    std::bind(
      &ParametersHandler::modifyParametersCallback, this,
      std::placeholders::_1));
}

void ParametersHandler::modifyParametersCallback(
  std::vector<rclcpp::Parameter> & parameters)
{
  bool found = false;
  for (auto & param : parameters) {
    const std::string & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }
    found = true;
    break;
  }

  if (found) {
    for (auto & pre_cb : pre_callbacks_) {
      pre_cb();
    }
  }
}

rcl_interfaces::msg::SetParametersResult
ParametersHandler::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  std::vector<rclcpp::Parameter> plugin_params;
  for (auto & param : parameters) {
    const std::string & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }
    plugin_params.push_back(param);
  }

  if (!plugin_params.empty()) {
    for (auto & param : plugin_params) {
      const std::string & param_name = param.get_name();
      if (auto callback = get_param_callbacks_.find(param_name);
        callback != get_param_callbacks_.end())
      {
        callback->second(param, result);
      }
    }
  }

  if (!result.successful) {
    RCLCPP_ERROR(logger_, result.reason.c_str());
  }
  return result;
}

void ParametersHandler::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(parameters_change_mutex_);
  std::vector<rclcpp::Parameter> plugin_params;
  for (auto & param : parameters) {
    const std::string & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }
    plugin_params.push_back(param);
  }

  if (!plugin_params.empty()) {
    for (auto & param : plugin_params) {
      const std::string & param_name = param.get_name();
      if (auto callback = get_post_callbacks_.find(param_name);
        callback != get_post_callbacks_.end())
      {
        callback->second(param);
      }
    }
    for (auto & post_cb : post_callbacks_) {
      post_cb();
    }
  }
}

}  // namespace mppi
