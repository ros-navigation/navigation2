#ifndef NAV2_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_
#define NAV2_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.hpp"
#include <Eigen/Dense>
#include <cmath>
#include "pluginlib/class_loader.hpp"
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;
namespace nav2_costmap_2d
{
/**
 * @class PluginContainerLayer
 * @brief Holds a list of plugins and applies them only to the specific layer
 */
class PluginContainerLayer : public CostmapLayer
{
public:
  /**
   * @brief A constructor
   */
  PluginContainerLayer();
  /**
   * @brief A destructor
   */
  virtual ~PluginContainerLayer();
  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();
  /**
   * @brief Update the bounds of the master costmap by this layer's update
   *dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j);
  virtual void onFootprintChanged();
  /** @brief Update the footprint to match size of the parent costmap. */
  virtual void matchSize();
  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate();
  /**
   * @brief Activate the layer
   */
  virtual void activate();
  /**
   * @brief Reset this costmap
   */
  virtual void reset();
  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable()
  {
    return true;
  }
  void addPlugin(std::shared_ptr<Layer> plugin, std::string layer_name);
  pluginlib::ClassLoader<Layer> plugin_loader_{"nav2_costmap_2d", "nav2_costmap_2d::Layer"};
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

private:
  /// @brief Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;
  bool rolling_window_;
  int combination_method_;
  std::vector<std::shared_ptr<Layer>> plugins_;
  std::vector<std::shared_ptr<Layer>> filters_;
  std::vector<std::string> default_plugins_;
  std::vector<std::string> default_types_;
  std::vector<std::string> plugin_names_;
  std::vector<std::string> plugin_types_;
  std::vector<std::string> filter_names_;
  std::vector<std::string> filter_types_;
  // primary_costmap_ is a bottom costmap used by plugins when costmap filters were enabled.
  // combined_costmap_ is a final costmap where all results produced by plugins and filters (if any)
  // to be merged.
  // The separation is aimed to avoid interferences of work between plugins and filters.
  // primay_costmap_ and combined_costmap_ have the same sizes, origins and default values.
  Costmap2D primary_costmap_, combined_costmap_;
};
} // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__PLUGIN_CONTAINER_LAYER_HPP_
