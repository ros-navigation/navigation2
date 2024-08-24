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
#include "nav2_costmap_2d/inflation_layer.hpp"

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_events_filter.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::InflationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

InflationLayer::InflationLayer()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
{
  access_ = new mutex_t();
}

InflationLayer::~InflationLayer()
{
  dyn_params_handler_.reset();
  delete access_;
}

void
InflationLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.55));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(10.0));
  declareParameter("inflate_unknown", rclcpp::ParameterValue(false));
  declareParameter("inflate_around_unknown", rclcpp::ParameterValue(false));

  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
    node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);
    node->get_parameter(name_ + "." + "inflate_unknown", inflate_unknown_);
    node->get_parameter(name_ + "." + "inflate_around_unknown", inflate_around_unknown_);

    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(
        &InflationLayer::dynamicParametersCallback,
        this, std::placeholders::_1));
  }

  current_ = true;
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  matchSize();
}

void
InflationLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  seen_ = std::vector<bool>(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), false);
}

void
InflationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (need_reinflation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void
InflationLayer::onFootprintChanged()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  need_reinflation_ = true;

  if (inflation_radius_ < inscribed_radius_) {
    RCLCPP_ERROR(
      logger_,
      "The configured inflation radius (%.3f) is smaller than "
      "the computed inscribed radius (%.3f) of your footprint, "
      "it is highly recommended to set inflation radius to be at "
      "least as big as the inscribed radius to avoid collisions",
      inflation_radius_, inscribed_radius_);
  }

  RCLCPP_DEBUG(
    logger_, "InflationLayer::onFootprintChanged(): num footprint points: %zu,"
    " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
    layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void
InflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_ || (cell_inflation_radius_ == 0)) {
    return;
  }

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  for (auto & dist : inflation_cells_) {
    RCLCPP_FATAL_EXPRESSION(
      logger_,
      !dist.empty(), "The inflation list must be empty at the beginning of inflation");
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_.size() != size_x * size_y) {
    RCLCPP_WARN(
      logger_, "InflationLayer::updateCosts(): seen_ vector size is wrong");
    seen_ = std::vector<bool>(size_x * size_y, false);
  }

  std::fill(begin(seen_), end(seen_), false);

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  min_i -= static_cast<int>(cell_inflation_radius_);
  min_j -= static_cast<int>(cell_inflation_radius_);
  max_i += static_cast<int>(cell_inflation_radius_);
  max_j += static_cast<int>(cell_inflation_radius_);

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Inflation list; we append cells to visit in a list associated with
  // its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before,
  // with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  auto & obs_bin = inflation_cells_[0];
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = static_cast<int>(master_grid.getIndex(i, j));
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
        obs_bin.emplace_back(index, i, j, i, j);
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the
  // corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  for (const auto & dist_bin : inflation_cells_) {
    for (std::size_t i = 0; i < dist_bin.size(); ++i) {
      // Do not use iterator or for-range based loops to
      // iterate though dist_bin, since it's size might
      // change when a new cell is enqueued, invalidating all iterators
      unsigned int index = dist_bin[i].index_;

      // ignore if already visited
      if (seen_[index]) {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = dist_bin[i].x_;
      unsigned int my = dist_bin[i].y_;
      unsigned int sx = dist_bin[i].src_x_;
      unsigned int sy = dist_bin[i].src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      // In order to avoid artifacts appeared out of boundary areas
      // when some layer is going after inflation_layer,
      // we need to apply inflation_layer only to inside of given bounds
      if (static_cast<int>(mx) >= base_min_i &&
        static_cast<int>(my) >= base_min_j &&
        static_cast<int>(mx) < base_max_i &&
        static_cast<int>(my) < base_max_j)
      {
        if (old_cost == NO_INFORMATION &&
          (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        {
          master_array[index] = cost;
        } else {
          master_array[index] = std::max(old_cost, cost);
        }
      }

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0) {
        enqueue(index - 1, mx - 1, my, sx, sy);
      }
      if (my > 0) {
        enqueue(index - size_x, mx, my - 1, sx, sy);
      }
      if (mx < size_x - 1) {
        enqueue(index + 1, mx + 1, my, sx, sy);
      }
      if (my < size_y - 1) {
        enqueue(index + size_x, mx, my + 1, sx, sy);
      }
    }
  }

  for (auto & dist : inflation_cells_) {
    dist.clear();
    dist.reserve(200);
  }

  current_ = true;
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
void
InflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    // we compute our distance table one cell further than the
    // inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within
    // the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance_matrix_[mx - src_x + r][my - src_y + r]].emplace_back(
      index, mx, my, src_x, src_y);
  }
}

void
InflationLayer::computeCaches()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (cell_inflation_radius_ == 0) {
    return;
  }

  cache_length_ = cell_inflation_radius_ + 2;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
    }
  }

  int max_dist = generateIntegerDistances();
  inflation_cells_.clear();
  inflation_cells_.resize(max_dist + 1);
  for (auto & dist : inflation_cells_) {
    dist.reserve(200);
  }
}

int
InflationLayer::generateIntegerDistances()
{
  const int r = cell_inflation_radius_ + 2;
  const int size = r * 2 + 1;

  std::vector<std::pair<int, int>> points;

  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }

  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );

  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      level++;
    }
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }

  distance_matrix_ = distance_matrix;
  return level;
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
InflationLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  bool need_cache_recompute = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "inflation_radius" &&
        inflation_radius_ != parameter.as_double())
      {
        inflation_radius_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      } else if (param_name == name_ + "." + "cost_scaling_factor" && // NOLINT
        cost_scaling_factor_ != parameter.as_double())
      {
        cost_scaling_factor_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        need_reinflation_ = true;
        current_ = false;
      } else if (param_name == name_ + "." + "inflate_unknown" && // NOLINT
        inflate_unknown_ != parameter.as_bool())
      {
        inflate_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      } else if (param_name == name_ + "." + "inflate_around_unknown" && // NOLINT
        inflate_around_unknown_ != parameter.as_bool())
      {
        inflate_around_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      }
    }
  }

  if (need_cache_recompute) {
    matchSize();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
