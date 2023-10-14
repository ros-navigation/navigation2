/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__LAYERED_COSTMAP_HPP_
#define NAV2_COSTMAP_2D__LAYERED_COSTMAP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a costmap
   */
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  /**
   * @brief  Destructor
   */
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.
   * If you want to update the map outside of the update loop that runs, you can call this.
   */
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  std::string getGlobalFrameID() const
  {
    return global_frame_;
  }

  /**
   * @brief Resize the map to a new size, resolution, or origin
   */
  void resizeMap(
    unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
    double origin_y,
    bool size_locked = false);

  /**
   * @brief Get the size of the bounds for update
   */
  void getUpdatedBounds(double & minx, double & miny, double & maxx, double & maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  /**
   * @brief If the costmap is current, e.g. are all the layers processing recent data
   * and not stale information for a good state.
   */
  bool isCurrent();

  /**
   * @brief Get the costmap pointer to the master costmap
   */
  Costmap2D * getCostmap()
  {
    return &combined_costmap_;
  }

  /**
   * @brief If this costmap is rolling or not
   */
  bool isRolling()
  {
    return rolling_window_;
  }

  /**
   * @brief If this costmap is tracking unknown space or not
   */
  bool isTrackingUnknown()
  {
    return combined_costmap_.getDefaultValue() == nav2_costmap_2d::NO_INFORMATION;
  }

  /**
   * @brief Get the vector of pointers to the costmap plugins
   */
  std::vector<std::shared_ptr<Layer>> * getPlugins()
  {
    return &plugins_;
  }

  /**
   * @brief Get the vector of pointers to the costmap filters
   */
  std::vector<std::shared_ptr<Layer>> * getFilters()
  {
    return &filters_;
  }

  /**
   * @brief Add a new plugin to the plugins vector to process
   */
  void addPlugin(std::shared_ptr<Layer> plugin);


  /**
   * @brief Add a new costmap filter plugin to the filters vector to process
   */
  void addFilter(std::shared_ptr<Layer> filter);


  /**
   * @brief Get if the size of the costmap is locked
   */
  bool isSizeLocked()
  {
    return size_locked_;
  }

  /**
   * @brief Get the bounds of the costmap
   */
  void getBounds(unsigned int * x0, unsigned int * xn, unsigned int * y0, unsigned int * yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

  /**
   * @brief if the costmap is initialized
   */
  bool isInitialized()
  {
    return initialized_;
  }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec);

  /** @brief Returns the latest footprint stored with setFootprint(). */
  const std::vector<geometry_msgs::msg::Point> & getFootprint() {return footprint_;}

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getCircumscribedRadius() {return circumscribed_radius_;}

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getInscribedRadius() {return inscribed_radius_;}

  /** @brief Checks if the robot is outside the bounds of its costmap in the case
  * of poorly configured setups. */
  bool isOutofBounds(double robot_x, double robot_y);

private:
  // primary_costmap_ is a bottom costmap used by plugins when costmap filters were enabled.
  // combined_costmap_ is a final costmap where all results produced by plugins and filters (if any)
  // to be merged.
  // The separation is aimed to avoid interferences of work between plugins and filters.
  // primay_costmap_ and combined_costmap_ have the same sizes, origins and default values.
  Costmap2D primary_costmap_, combined_costmap_;
  std::string global_frame_;

  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

  bool current_;
  double minx_, miny_, maxx_, maxy_;
  unsigned int bx0_, bxn_, by0_, byn_;

  std::vector<std::shared_ptr<Layer>> plugins_;
  std::vector<std::shared_ptr<Layer>> filters_;

  bool initialized_;
  bool size_locked_;
  double circumscribed_radius_, inscribed_radius_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__LAYERED_COSTMAP_HPP_
