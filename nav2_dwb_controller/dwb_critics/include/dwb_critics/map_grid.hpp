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

#ifndef DWB_CRITICS__MAP_GRID_HPP_
#define DWB_CRITICS__MAP_GRID_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "dwb_core/trajectory_critic.hpp"
#include "costmap_queue/costmap_queue.hpp"

namespace dwb_critics
{
/**
 * @class MapGridCritic
 * @brief breadth-first scoring of all the cells in the costmap
 *
 * This TrajectoryCritic assigns a score to every cell in the costmap based on
 * the distance to the cell from some set of source points. The cells corresponding
 * with the source points are marked with some initial score, and then every other cell
 * is updated with a score based on its relation to the closest source cell, based on a
 * breadth-first exploration of the cells of the costmap.
 *
 * This approach was chosen for computational efficiency, such that each trajectory
 * need not be compared to the list of source points.
 */
class MapGridCritic : public dwb_core::TrajectoryCritic
{
public:
  // Standard TrajectoryCritic Interface
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>> & cost_channels) override;
  double getScale() const override {return costmap_->getResolution() * 0.5 * scale_;}

  // Helper Functions
  /**
   * @brief Retrieve the score for a single pose
   * @param pose The pose to score, assumed to be in the same frame as the costmap
   * @return The score associated with the cell of the costmap where the pose lies
   */
  virtual double scorePose(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Retrieve the score for a particular cell of the costmap
   * @param x x-coordinate within the costmap
   * @param y y-coordinate within the costmap
   * @return the score associated with that cell.
   */
  inline double getScore(unsigned int x, unsigned int y)
  {
    return cell_values_[costmap_->getIndex(x, y)];
  }

  /**
   * @brief Sets the score of a particular cell to the obstacle cost
   * @param index Index of the cell to mark
   */
  void setAsObstacle(unsigned int index);

protected:
  /**
   * @brief Separate modes for aggregating scores across the multiple poses in a trajectory.
   *
   * Last returns the score associated with the last pose in the trajectory
   * Sum returns the sum of all the scores
   * Product returns the product of all the (non-zero) scores
   */
  // cppcheck-suppress syntaxError
  enum class ScoreAggregationType {Last, Sum, Product};

  /**
   * @class MapGridQueue
   * @brief Subclass of CostmapQueue that avoids Obstacles and Unknown Values
   */
  class MapGridQueue : public costmap_queue::CostmapQueue
  {
public:
    MapGridQueue(nav2_costmap_2d::Costmap2D & costmap, MapGridCritic & parent)
    : costmap_queue::CostmapQueue(costmap, true), parent_(parent) {}
    virtual ~MapGridQueue() = default;
    bool validCellToQueue(const costmap_queue::CellData & cell) override;

    /**
     * @brief Clear the queue and stop rejecting obstacles
     */
    void reset() override;

    /**
     * @brief Set whether validCellToQueue rejects lethal, inscribed and unknown cells
     * @param avoid_obstacles True to reject those cells and score them as obstacles
     */
    void setAvoidObstacles(bool avoid_obstacles) {avoid_obstacles_ = avoid_obstacles;}

    /**
     * @brief Get the next cell in order of steps walked from its source, and enqueue its neighbors
     * @return The next cell, its distance_ holding the steps walked from its source
     *
     * A cell rejected by validCellToQueue is returned without enqueuing its neighbors.
     * getNextCell orders cells by their straight line distance to the source, which is not
     * the distance walked once obstacles are rejected.
     * NB: Assumes that isEmpty has been called before this call and returned false
     */
    costmap_queue::CellData getNextCellAroundObstacles();

protected:
    /**
     * @brief Enqueue a neighbor of a cell one step further from its source, if not seen yet
     * @param from The cell being expanded
     * @param x The x coordinate of the neighbor
     * @param y The y coordinate of the neighbor
     */
    void enqueueNeighbor(const costmap_queue::CellData & from, unsigned int x, unsigned int y);

    MapGridCritic & parent_;
    bool avoid_obstacles_{false};
  };

  /**
   * @brief Clear the queueDWB_CRITICS_MAP_GRID_He and set cell_values_ to the appropriate number of unreachableCellScore
   */
  void reset() override;

  /**
   * @brief Go through the queue and set the cells to the Manhattan distance from their parents
   */
  void propagateManhattanDistances();

  /**
   * @brief Set the cells to their distance from the queued seeds
   *
   * With obstacle_aware_propagation the distance goes around obstacles. If that does not reach
   * the robot, as it is blocked or no seed is free, this uses propagateManhattanDistances.
   *
   * @param robot_pose The pose of the robot, assumed to be in the same frame as the costmap
   */
  void propagateDistances(const geometry_msgs::msg::Pose & robot_pose);

  /**
   * @brief Set the cells to their 4-connected distance around obstacles to the nearest seed
   * @param robot_pose The pose of the robot, assumed to be in the same frame as the costmap
   * @return True if the search reached the cell of the robot
   */
  bool propagateAroundObstacles(const geometry_msgs::msg::Pose & robot_pose);

  std::shared_ptr<MapGridQueue> queue_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::vector<double> cell_values_;
  double obstacle_score_, unreachable_score_;  ///< Special cell_values
  bool stop_on_failure_;
  bool obstacle_aware_propagation_;
  ScoreAggregationType aggregationType_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS__MAP_GRID_HPP_
