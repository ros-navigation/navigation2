// Copyright 2024 Nav2 Contributors
// Licensed under the Apache License, Version 2.0

#ifndef EXAMPLE_COSTMAP_LAYER__GRADIENT_LAYER_HPP_
#define EXAMPLE_COSTMAP_LAYER__GRADIENT_LAYER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace example_costmap_layer
{

/**
 * @class GradientLayer
 * @brief A costmap layer that adds gradient costs around a configurable point
 *
 * This example layer demonstrates how to implement a custom costmap layer.
 * It creates a cost gradient centered at a specified point, which can be
 * useful for implementing keep-out zones, preferred travel regions, or
 * other cost-based navigation behaviors.
 */
class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  /**
   * @brief Default constructor
   */
  GradientLayer();

  /**
   * @brief Virtual destructor
   */
  virtual ~GradientLayer() = default;

  /**
   * @brief Called after initialize() to allow subclass-specific initialization
   */
  void onInitialize() override;

  /**
   * @brief Activate any publishers or threads
   */
  void activate() override;

  /**
   * @brief Deactivate publishers and threads
   */
  void deactivate() override;

  /**
   * @brief Reset layer to initial state
   */
  void reset() override;

  /**
   * @brief Query if this layer can be cleared
   */
  bool isClearable() override;

  /**
   * @brief Update the bounds of the area to be updated
   * @param robot_x Robot X position in world coordinates
   * @param robot_y Robot Y position in world coordinates
   * @param robot_yaw Robot orientation
   * @param min_x Minimum X bound (output)
   * @param min_y Minimum Y bound (output)
   * @param max_x Maximum X bound (output)
   * @param max_y Maximum Y bound (output)
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
   * @brief Update costs in the master costmap
   * @param master_grid Reference to the master costmap
   * @param min_i Minimum X cell index
   * @param min_j Minimum Y cell index
   * @param max_i Maximum X cell index
   * @param max_j Maximum Y cell index
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;

  /**
   * @brief Called when costmap is resized
   */
  void matchSize() override;

  /**
   * @brief Handle footprint changes
   */
  void onFootprintChanged() override;

private:
  /**
   * @brief Calculate cost at a given distance from the gradient center
   * @param distance Distance from center in meters
   * @return Cost value (0-254)
   */
  unsigned char calculateCost(double distance);

  // Parameters
  double gradient_center_x_;
  double gradient_center_y_;
  double gradient_radius_;
  double gradient_strength_;
  bool enabled_;

  // Need full map update flag
  bool need_recalculation_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("GradientLayer")};
};

}  // namespace example_costmap_layer

#endif  // EXAMPLE_COSTMAP_LAYER__GRADIENT_LAYER_HPP_
