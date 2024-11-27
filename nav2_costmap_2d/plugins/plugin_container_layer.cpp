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
PluginContainerLayer::PluginContainerLayer()
: default_plugins_{"static_layer", "obstacle_layer", "inflation_layer"},
  default_types_{
    "nav2_costmap_2d::StaticLayer",
    "nav2_costmap_2d::ObstacleLayer",
    "nav2_costmap_2d::InflationLayer"}
{
  costmap_ = NULL;
}

PluginContainerLayer::~PluginContainerLayer()
{}

void PluginContainerLayer::onInitialize()
{

  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  primary_costmap_.setDefaultValue(0);
  combined_costmap_.setDefaultValue(0);

  node->declare_parameter(name_ + "." + "enabled", rclcpp::ParameterValue(false));
  node->declare_parameter(name_ + "." + "plugins", rclcpp::ParameterValue(default_plugins_));
  node->declare_parameter(name_ + "." + "combination_method", rclcpp::ParameterValue(0));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "plugins", plugin_names_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);

  plugin_types_.resize(plugin_names_.size());
  filter_types_.resize(filter_names_.size());

  for (size_t i = 0; i < plugin_names_.size(); ++i) {
    plugin_types_[i] = nav2_util::get_plugin_type_param(node, name_ + "." + plugin_names_[i]);
  }
  for (size_t i = 0; i < filter_names_.size(); ++i) {
    filter_types_[i] = nav2_util::get_plugin_type_param(node, name_ + "." + filter_names_[i]);
  }

  // Then load and add the plug-ins to the costmap
  for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]); //this is a shared pointer to a Layer called plugin
    std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));
    addPlugin(plugin);
    plugin->initialize(
      layered_costmap_, name_ + "." + plugin_names_[i], tf_, node,
      callback_group_);
    lock.unlock();
  }

  PluginContainerLayer::matchSize();
  current_ = true;

}

void PluginContainerLayer::addPlugin(std::shared_ptr<Layer> plugin)
{
  plugins_.push_back(plugin);
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

  //1. clear combined map
  combined_costmap_.resetMap(min_i, min_j, max_i, max_j);
  //2. update the internal costmap with the plugins associated with the layer

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->updateCosts(combined_costmap_, min_i, min_j, max_i, max_j);
  }
  //3. point the private member costmap to the char map of the Costmap2D
  costmap_ = combined_costmap_.getCharMap(); //This will point to the CharMap...this feels memory leaky

  //4. update master grid depending on what method we pick
  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default: // Nothing
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
  primary_costmap_.resizeMap(size_x_, size_y_, resolution_, origin_x_, origin_y_); //Where do I get these numbers
  combined_costmap_.resizeMap(size_x_, size_y_, resolution_, origin_x_, origin_y_); //Where do I get these numbers

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->matchSize();
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
        combination_method_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

} // namespace nav2_costmap_2d
