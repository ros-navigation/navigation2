// Copyright (c) 2024 Polymath Robotics, Inc.
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

#include "nav2_costmap_2d/plugin_container_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::PluginContainerLayer, nav2_costmap_2d::Layer)

using std::vector;

namespace nav2_costmap_2d
{

void PluginContainerLayer::onInitialize()
{
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, name_ + "." + "enabled",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name_ + "." + "plugins",
      rclcpp::ParameterValue(std::vector<std::string>{}));
  nav2_util::declare_parameter_if_not_declared(node, name_ + "." + "combination_method",
      rclcpp::ParameterValue(1));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "plugins", plugin_names_);

  int combination_method_param{};
  node->get_parameter(name_ + "." + "combination_method", combination_method_param);
  combination_method_ = combination_method_from_int(combination_method_param);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &PluginContainerLayer::dynamicParametersCallback,
      this,
      std::placeholders::_1));

  plugin_types_.resize(plugin_names_.size());

  for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
    plugin_types_[i] = nav2_util::get_plugin_type_param(node, name_ + "." + plugin_names_[i]);
    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]);
    addPlugin(plugin, plugin_names_[i]);
  }

  default_value_ = nav2_costmap_2d::NO_INFORMATION;

  PluginContainerLayer::matchSize();
  current_ = true;
}

void PluginContainerLayer::addPlugin(std::shared_ptr<Layer> plugin, std::string layer_name)
{
  plugins_.push_back(plugin);
  auto node = node_.lock();
  plugin->initialize(layered_costmap_, name_ + "." + layer_name, tf_, node, callback_group_);
}

void PluginContainerLayer::updateBounds(
  double robot_x,
  double robot_y,
  double robot_yaw,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }
}

void PluginContainerLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->updateCosts(*this, min_i, min_j, max_i, max_j);
  }

  switch (combination_method_) {
    case CombinationMethod::Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::Max:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::MaxWithoutUnknownOverwrite:
      updateWithMaxWithoutUnknownOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }

  current_ = true;
}

void PluginContainerLayer::activate()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->activate();
  }
}

void PluginContainerLayer::deactivate()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->deactivate();
  }
}

void PluginContainerLayer::reset()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->reset();
  }
  resetMaps();
  current_ = false;
}

void PluginContainerLayer::onFootprintChanged()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

void PluginContainerLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(),
    master->getResolution(), master->getOriginX(), master->getOriginY());

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->matchSize();
  }
}

bool PluginContainerLayer::isClearable()
{
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    if((*plugin)->isClearable()) {
      return true;
    }
  }
  return false;
}

void PluginContainerLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert)
{
  CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    auto costmap_layer = std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(*plugin);
    if ((*plugin)->isClearable() && costmap_layer != nullptr) {
      costmap_layer->clearArea(start_x, start_y, end_x, end_y, invert);
    }
  }
}

rcl_interfaces::msg::SetParametersResult PluginContainerLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "combination_method") {
        combination_method_ = combination_method_from_int(parameter.as_int());
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
