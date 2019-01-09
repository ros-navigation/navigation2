/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_
#define COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "costmap_queue/map_based_queue.hpp"

namespace costmap_queue
{
/**
 * @class CellData
 * @brief Storage for cell information used during queue expansion
 */
class CellData
{
public:
  /**
   * @brief Real Constructor
   * @param d The distance to the nearest obstacle
   * @param i The index of the cell in the costmap. Redundant with the following two parameters.
   * @param x The x coordinate of the cell in the cost map
   * @param y The y coordinate of the cell in the cost map
   * @param sx The x coordinate of the closest source cell in the costmap
   * @param sy The y coordinate of the closest source cell in the costmap
   */
  CellData(
    const double d, const unsigned int i, const unsigned int x, const unsigned int y,
    const unsigned int sx, const unsigned int sy)
  : distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }

  /**
   * @brief Default Constructor - Should be used sparingly
   */
  CellData()
  : distance_(std::numeric_limits<double>::max()), index_(0), x_(0), y_(0), src_x_(0), src_y_(0)
  {
  }

  static unsigned absolute_difference(const unsigned x, const unsigned y)
  {
    return (x > y) ? (x - y) : (y - x);
  }

  double distance_;
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @class CostmaQueue
 * @brief A tool for finding the cells closest to some set of originating cells.
 *
 * A common operation with costmaps is to define a set of cells in the costmap, and then
 * perform some operation on all the other cells based on which cell in the original set
 * the other cells are closest to. This operation is done in the inflation layer to figure out
 * how far each cell is from an obstacle, and is also used in a number of Trajectory cost functions.
 *
 * It is implemented with a queue. The standard operation is to enqueueCell the original set, and then
 * retreive the other cells with the isEmpty/getNextCell iterator-like functionality. getNextCell
 * returns an object that contains the coordinates of this cell and the origin cell, as well as
 * the distance between them. By default, the Euclidean distance is used for ordering, but passing in
 * manhattan=true to the constructor will use the Manhattan distance.
 *
 * The validCellToQueue overridable-function allows for deriving classes to limit the queue traversal
 * to a subset of all costmap cells. LimitedCostmapQueue does this by ignoring distances above a limit.
 *
 */
class CostmapQueue : public MapBasedQueue<CellData>
{
public:
  /**
   * @brief constructor
   * @param costmap Costmap which defines the size/number of cells
   * @param manhattan If true, sort cells by Manhattan distance, otherwise use Euclidean distance
   */
  explicit CostmapQueue(nav2_costmap_2d::Costmap2D & costmap, bool manhattan = false);

  /**
   * @brief Clear the queue
   */
  void reset() override;

  /**
   * @brief Add a cell the queue
   * @param x X coordinate of the cell
   * @param y Y coordinate of the cell
   */
  void enqueueCell(unsigned int x, unsigned int y);

  /**
   * @brief Get the next cell to examine, and enqueue its neighbors as needed
   * @return The next cell
   *
   * NB: Assumes that isEmpty has been called before this call and returned false
   */
  CellData getNextCell();

  /**
   * @brief Check to see if we should add this cell to the queue. Always true unless overridden.
   * @param cell The cell to check
   * @return True, unless overriden
   */
  virtual bool validCellToQueue(const CellData & /*cell*/) {return true;}
  /**
   * @brief convenience typedef for a pointer
   */
  typedef std::shared_ptr<CostmapQueue> Ptr;

protected:
  /**
   * @brief Enqueue a cell with the given coordinates and the given source cell
   */
  void enqueueCell(
    unsigned int index, unsigned int cur_x, unsigned int cur_y, unsigned int src_x,
    unsigned int src_y);

  /**
   * @brief Compute the cached distances
   */
  void computeCache();

  nav2_costmap_2d::Costmap2D & costmap_;
  std::vector<bool> seen_;
  int max_distance_;
  bool manhattan_;

protected:
  /**
   * @brief  Lookup pre-computed distances
   * @param cur_x The x coordinate of the current cell
   * @param cur_y The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(
    const unsigned int cur_x, const unsigned int cur_y,
    const unsigned int src_x, const unsigned int src_y)
  {
    unsigned int dx = CellData::absolute_difference(cur_x, src_x);
    unsigned int dy = CellData::absolute_difference(cur_y, src_y);
    return cached_distances_[dx][dy];
  }
  std::vector<std::vector<double>> cached_distances_;
  int cached_max_distance_;
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE__COSTMAP_QUEUE_HPP_
