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
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "nav2_costmap_2d/footprint.hpp"


using std::vector;

namespace nav2_costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
: costmap_(),
  global_frame_(global_frame),
  rolling_window_(rolling_window),
  current_(false),
  minx_(0.0),
  miny_(0.0),
  maxx_(0.0),
  maxy_(0.0),
  bx0_(0),
  bxn_(0),
  by0_(0),
  byn_(0),
  initialized_(false),
  size_locked_(false),
  circumscribed_radius_(1.0),
  inscribed_radius_(0.1)
{
  if (track_unknown) {
    costmap_.setDefaultValue(255);
  } else {
    costmap_.setDefaultValue(0);
  }
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0) {
    plugins_.pop_back();
  }
}

void LayeredCostmap::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x,
  double origin_y,
  bool size_locked)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    (*plugin)->matchSize();
  }
}

bool LayeredCostmap::isOutofBounds(double robot_x, double robot_y)
{
  unsigned int mx, my;
  return !costmap_.worldToMap(robot_x, robot_y, mx, my);
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

  // if we're using a rolling buffer costmap...
  // we need to update the origin using the robot's position
  if (rolling_window_) {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (isOutofBounds(robot_x, robot_y)) {
    RCLCPP_WARN(
      rclcpp::get_logger("nav2_costmap_2d"),
      "Robot is out of bounds of the costmap!");
  }

  if (plugins_.size() == 0) {
    return;
  }

  minx_ = miny_ = std::numeric_limits<double>::max();
  maxx_ = maxy_ = std::numeric_limits<double>::lowest();

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "nav2_costmap_2d"), "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy,
        minx_, miny_, maxx_, maxy_,
        (*plugin)->getName().c_str());
    }
  }

  int x0, xn, y0, yn;
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(static_cast<int>(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(static_cast<int>(costmap_.getSizeInCellsY()), yn + 1);

  RCLCPP_DEBUG(
    rclcpp::get_logger(
      "nav2_costmap_2d"), "Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0) {
    return;
  }

  costmap_.resetMap(x0, y0, xn, yn);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec)
{
  footprint_ = footprint_spec;
  nav2_costmap_2d::calculateMinAndMaxDistances(
    footprint_spec,
    inscribed_radius_, circumscribed_radius_);

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin();
    plugin != plugins_.end();
    ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace nav2_costmap_2d
