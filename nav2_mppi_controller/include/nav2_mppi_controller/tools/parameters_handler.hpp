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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_

#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mppi
{
/**
 * @class Parameter Type enum
 */
enum class ParameterType { Dynamic, Static };

/**
 * @class mppi::ParametersHandler
 * @brief Handles getting parameters and dynamic parmaeter changes
 */
class ParametersHandler
{
public:
  using get_param_func_t = void (const rclcpp::Parameter & param);
  using post_callback_t = void ();
  using pre_callback_t = void ();

  /**
    * @brief Constructor for mppi::ParametersHandler
    */
  ParametersHandler() = default;

  /**
    * @brief Constructor for mppi::ParametersHandler
    * @param parent Weak ptr to node
    */
  explicit ParametersHandler(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

  /**
    * @brief Starts processing dynamic parameter changes
    */
  void start();

  /**
    * @brief Dynamic parameter callback
    * @param parameter Parameter changes to process
    * @return Set Parameter Result
    */
  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
    * @brief Get an object to retreive parameters
    * @param ns Namespace to get parameters within
    * @return Parameter getter object
    */
  inline auto getParamGetter(const std::string & ns);

  /**
    * @brief Set a callback to process after parameter changes
    * @param callback Callback function
    */
  template<typename T>
  void addPostCallback(T && callback);

  /**
    * @brief Set a callback to process before parameter changes
    * @param callback Callback function
    */
  template<typename T>
  void addPreCallback(T && callback);

  /**
    * @brief Set a parameter to a dynamic parameter callback
    * @param setting Parameter
    * @param name Name of parameter
    */
  template<typename T>
  void setDynamicParamCallback(T & setting, const std::string & name);

  /**
    * @brief Get mutex lock for changing parameters
    * @return Pointer to mutex
    */
  std::mutex * getLock()
  {
    return &parameters_change_mutex_;
  }

  /**
    * @brief Set a parameter to a dynamic parameter callback
    * @param name Name of parameter
    * @param callback Parameter callback
    */
  template<typename T>
  void addDynamicParamCallback(const std::string & name, T && callback);

protected:
  /**
    * @brief Gets parameter
    * @param setting Return Parameter type
    * @param name Parameter name
    * @param default_value Default parameter value
    * @param param_type Type of parameter (dynamic or static)
    */
  template<typename SettingT, typename ParamT>
  void getParam(
    SettingT & setting, const std::string & name, ParamT default_value,
    ParameterType param_type = ParameterType::Dynamic);

  /**
    * @brief Set a parameter
    * @param setting Return Parameter type
    * @param name Parameter name
    * @param node Node to set parameter via
    */
  template<typename ParamT, typename SettingT, typename NodeT>
  void setParam(SettingT & setting, const std::string & name, NodeT node) const;

  /**
    * @brief Converts parameter type to real types
    * @param parameter Parameter to convert into real type
    * @return parameter as a functional type
    */
  template<typename T>
  static auto as(const rclcpp::Parameter & parameter);

  std::mutex parameters_change_mutex_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_param_handler_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string node_name_;

  bool verbose_{false};

  std::unordered_map<std::string, std::function<get_param_func_t>>
  get_param_callbacks_;

  std::vector<std::function<pre_callback_t>> pre_callbacks_;
  std::vector<std::function<post_callback_t>> post_callbacks_;
};

inline auto ParametersHandler::getParamGetter(const std::string & ns)
{
  return [this, ns](
    auto & setting, const std::string & name, auto default_value,
    ParameterType param_type = ParameterType::Dynamic) {
           getParam(
             setting, ns.empty() ? name : ns + "." + name,
             std::move(default_value), param_type);
         };
}

template<typename T>
void ParametersHandler::addDynamicParamCallback(const std::string & name, T && callback)
{
  get_param_callbacks_[name] = callback;
}

template<typename T>
void ParametersHandler::addPostCallback(T && callback)
{
  post_callbacks_.push_back(callback);
}

template<typename T>
void ParametersHandler::addPreCallback(T && callback)
{
  pre_callbacks_.push_back(callback);
}

template<typename SettingT, typename ParamT>
void ParametersHandler::getParam(
  SettingT & setting, const std::string & name,
  ParamT default_value,
  ParameterType param_type)
{
  auto node = node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));

  setParam<ParamT>(setting, name, node);

  if (param_type == ParameterType::Dynamic) {
    setDynamicParamCallback(setting, name);
  }
}

template<typename ParamT, typename SettingT, typename NodeT>
void ParametersHandler::setParam(
  SettingT & setting, const std::string & name, NodeT node) const
{
  ParamT param_in{};
  node->get_parameter(name, param_in);
  setting = static_cast<SettingT>(param_in);
}

template<typename T>
void ParametersHandler::setDynamicParamCallback(T & setting, const std::string & name)
{
  if (get_param_callbacks_.find(name) != get_param_callbacks_.end()) {
    return;
  }

  auto callback = [this, &setting, name](const rclcpp::Parameter & param) {
      setting = as<T>(param);

      if (verbose_) {
        RCLCPP_INFO(logger_, "Dynamic parameter changed: %s", std::to_string(param).c_str());
      }
    };

  addDynamicParamCallback(name, callback);

  if (verbose_) {
    RCLCPP_INFO(logger_, "Dynamic Parameter added %s", name.c_str());
  }
}

template<typename T>
auto ParametersHandler::as(const rclcpp::Parameter & parameter)
{
  if constexpr (std::is_same_v<T, bool>) {
    return parameter.as_bool();
  } else if constexpr (std::is_integral_v<T>) {
    return parameter.as_int();
  } else if constexpr (std::is_floating_point_v<T>) {
    return parameter.as_double();
  } else if constexpr (std::is_same_v<T, std::string>) {
    return parameter.as_string();
  } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) {
    return parameter.as_integer_array();
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    return parameter.as_double_array();
  } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    return parameter.as_string_array();
  } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
    return parameter.as_bool_array();
  }
}

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_
