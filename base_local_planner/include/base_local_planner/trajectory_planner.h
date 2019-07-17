/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_H_

#include <vector>
#include <cmath>

//for obstacle data access
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <base_local_planner/footprint_helper.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/trajectory.h>
#include <nav2_msgs/msg/position2_d_int.hpp>

// #include <base_local_planner/BaseLocalPlannerConfig.h>

//we'll take in a path as a vector of poses
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

//for creating a local cost grid
#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>

namespace base_local_planner {
  /**
   * @class TrajectoryPlanner
   * @brief Computes control velocities for a robot given a costmap, a plan, and the robot's position in the world.
   */
  class TrajectoryPlanner{
    friend class TrajectoryPlannerTest; //Need this for gtest to work
    public:
      /**
       * @brief  Constructs a trajectory controller
       * @param world_model The WorldModel the trajectory controller uses to check for collisions
       * @param costmap A reference to the Costmap the controller should use
       * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
       * @param inscribed_radius The radius of the inscribed circle of the robot
       * @param circumscribed_radius The radius of the circumscribed circle of the robot
       * @param acc_lim_x The acceleration limit of the robot in the x direction
       * @param acc_lim_y The acceleration limit of the robot in the y direction
       * @param acc_lim_theta The acceleration limit of the robot in the theta direction
       * @param sim_time The number of seconds to "roll-out" each trajectory
       * @param sim_granularity The distance between simulation points should be small enough that the robot doesn't hit things
       * @param vx_samples The number of trajectories to sample in the x dimension
       * @param vtheta_samples The number of trajectories to sample in the theta dimension
       * @param pdist_scale A scaling factor for how close the robot should stay to the path
       * @param gdist_scale A scaling factor for how aggresively the robot should pursue a local goal
       * @param occdist_scale A scaling factor for how much the robot should prefer to stay away from obstacles
       * @param heading_lookahead How far the robot should look ahead of itself when differentiating between different rotational velocities
       * @param oscillation_reset_dist The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
       * @param escape_reset_dist The distance the robot must travel before it can exit escape mode
       * @param escape_reset_theta The distance the robot must rotate before it can exit escape mode
       * @param holonomic_robot Set this to true if the robot being controlled can take y velocities and false otherwise
       * @param max_vel_x The maximum x velocity the controller will explore
       * @param min_vel_x The minimum x velocity the controller will explore
       * @param max_vel_th The maximum rotational velocity the controller will explore
       * @param min_vel_th The minimum rotational velocity the controller will explore
       * @param min_in_place_vel_th The absolute value of the minimum in-place rotational velocity the controller will explore
       * @param backup_vel The velocity to use while backing up
       * @param dwa Set this to true to use the Dynamic Window Approach, false to use acceleration limits
       * @param heading_scoring Set this to true to score trajectories based on the robot's heading after 1 timestep
       * @param heading_scoring_timestep How far to look ahead in time when we score heading based trajectories
       * @param meter_scoring adapt parameters to costmap resolution
       * @param simple_attractor Set this to true to allow simple attraction to a goal point instead of intelligent cost propagation
       * @param y_vels A vector of the y velocities the controller will explore
       * @param angular_sim_granularity The distance between simulation points for angular velocity should be small enough that the robot doesn't hit things
       */
      TrajectoryPlanner(WorldModel& world_model,
          const nav2_costmap_2d::Costmap2D& costmap,
          std::vector<geometry_msgs::msg::Point> footprint_spec,
          double acc_lim_x = 1.0, double acc_lim_y = 1.0, double acc_lim_theta = 1.0,
          double sim_time = 1.0, double sim_granularity = 0.025,
          int vx_samples = 20, int vtheta_samples = 20,
          double pdist_scale = 0.6, double gdist_scale = 0.8, double occdist_scale = 0.2,
          double heading_lookahead = 0.325, double oscillation_reset_dist = 0.05,
          double escape_reset_dist = 0.10, double escape_reset_theta = M_PI_2,
          bool holonomic_robot = true,
          double max_vel_x = 0.5, double min_vel_x = 0.1,
          double max_vel_th = 1.0, double min_vel_th = -1.0, double min_in_place_vel_th = 0.4,
          double backup_vel = -0.1,
          bool dwa = false, bool heading_scoring = false, double heading_scoring_timestep = 0.1,
          bool meter_scoring = true,
          bool simple_attractor = false,
          std::vector<double> y_vels = std::vector<double>(0),
          double stop_time_buffer = 0.2,
          double sim_period = 0.1, double angular_sim_granularity = 0.025);

      /**
       * @brief  Destructs a trajectory controller
       */
      ~TrajectoryPlanner();

      /**
       * @brief Reconfigures the trajectory planner
       */

      //void reconfigure(BaseLocalPlannerConfig &cfg);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
       * @param global_pose The current pose of the robot in world space
       * @param global_vel The current velocity of the robot in world space
       * @param drive_velocities Will be set to velocities to send to the robot base
       * @return The selected path or trajectory
       */
      Trajectory findBestPath(const geometry_msgs::msg::PoseStamped& global_pose,
                              geometry_msgs::msg::PoseStamped& global_vel, geometry_msgs::msg::PoseStamped& drive_velocities);

      /**
       * @brief  Update the plan that the controller is following
       * @param new_plan A new plan for the controller to follow
       * @param compute_dists Wheter or not to compute path/goal distances when a plan is updated
       */
      void updatePlan(const std::vector<geometry_msgs::msg::PoseStamped>& new_plan, bool compute_dists = false);

      /**
       * @brief  Accessor for the goal the robot is currently pursuing in world corrdinates
       * @param x Will be set to the x position of the local goal
       * @param y Will be set to the y position of the local goal
       */
      void getLocalGoal(double& x, double& y);

      /**
       * @brief  Generate and score a single trajectory
       * @param x The x position of the robot
       * @param y The y position of the robot
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @return True if the trajectory is legal, false otherwise
       */
      bool checkTrajectory(double x, double y, double theta, double vx, double vy,
          double vtheta, double vx_samp, double vy_samp, double vtheta_samp);

      /**
       * @brief  Generate and score a single trajectory
       * @param x The x position of the robot
       * @param y The y position of the robot
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @return The score (as double)
       */
      double scoreTrajectory(double x, double y, double theta, double vx, double vy,
          double vtheta, double vx_samp, double vy_samp, double vtheta_samp);

      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /** @brief Set the footprint specification of the robot. */
      void setFootprint( std::vector<geometry_msgs::msg::Point> footprint ) { footprint_spec_ = footprint; }

      /** @brief Return the footprint specification of the robot. */
      geometry_msgs::msg::Polygon getFootprintPolygon() const { return nav2_costmap_2d::toPolygon(footprint_spec_); }
      std::vector<geometry_msgs::msg::Point> getFootprint() const { return footprint_spec_; }

    private:
      /**
       * @brief  Create the trajectories we wish to explore, score them, and return the best option
       * @param x The x position of the robot
       * @param y The y position of the robot
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param acc_x The x acceleration limit of the robot
       * @param acc_y The y acceleration limit of the robot
       * @param acc_theta The theta acceleration limit of the robot
       * @return
       */
      Trajectory createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta,
          double acc_x, double acc_y, double acc_theta);

      /**
       * @brief  Generate and score a single trajectory
       * @param x The x position of the robot
       * @param y The y position of the robot
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @param acc_x The x acceleration limit of the robot
       * @param acc_y The y acceleration limit of the robot
       * @param acc_theta The theta acceleration limit of the robot
       * @param impossible_cost The cost value of a cell in the local map grid that is considered impassable
       * @param traj Will be set to the generated trajectory with its associated score
       */
      void generateTrajectory(double x, double y, double theta, double vx, double vy,
          double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y,
          double acc_theta, double impossible_cost, Trajectory& traj);

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @return
       */
      double footprintCost(double x_i, double y_i, double theta_i);

      base_local_planner::FootprintHelper footprint_helper_;

      MapGrid path_map_; ///< @brief The local map grid where we propagate path distance
      MapGrid goal_map_; ///< @brief The local map grid where we propagate goal distance
      const nav2_costmap_2d::Costmap2D& costmap_; ///< @brief Provides access to cost map information
      WorldModel& world_model_; ///< @brief The world model that the controller uses for collision detection

      std::vector<geometry_msgs::msg::Point> footprint_spec_; ///< @brief The footprint specification of the robot

      std::vector<geometry_msgs::msg::PoseStamped> global_plan_; ///< @brief The global path for the robot to follow

      bool stuck_left, stuck_right; ///< @brief Booleans to keep the robot from oscillating during rotation
      bool rotating_left, rotating_right; ///< @brief Booleans to keep track of the direction of rotation for the robot

      bool stuck_left_strafe, stuck_right_strafe; ///< @brief Booleans to keep the robot from oscillating during strafing
      bool strafe_right, strafe_left; ///< @brief Booleans to keep track of strafe direction for the robot

      bool escaping_; ///< @brief Boolean to keep track of whether we're in escape mode
      bool meter_scoring_;

      double goal_x_,goal_y_; ///< @brief Storage for the local goal the robot is pursuing

      double final_goal_x_, final_goal_y_; ///< @brief The end position of the plan.
      bool final_goal_position_valid_; ///< @brief True if final_goal_x_ and final_goal_y_ have valid data.  Only false if an empty path is sent.

      double sim_time_; ///< @brief The number of seconds each trajectory is "rolled-out"
      double sim_granularity_; ///< @brief The distance between simulation points
      double angular_sim_granularity_; ///< @brief The distance between angular simulation points

      int vx_samples_; ///< @brief The number of samples we'll take in the x dimenstion of the control space
      int vtheta_samples_; ///< @brief The number of samples we'll take in the theta dimension of the control space

      double pdist_scale_, gdist_scale_, occdist_scale_; ///< @brief Scaling factors for the controller's cost function
      double acc_lim_x_, acc_lim_y_, acc_lim_theta_; ///< @brief The acceleration limits of the robot

      double prev_x_, prev_y_; ///< @brief Used to calculate the distance the robot has traveled before reseting oscillation booleans
      double escape_x_, escape_y_, escape_theta_; ///< @brief Used to calculate the distance the robot has traveled before reseting escape booleans

      Trajectory traj_one, traj_two; ///< @brief Used for scoring trajectories

      double heading_lookahead_; ///< @brief How far the robot should look ahead of itself when differentiating between different rotational velocities
      double oscillation_reset_dist_; ///< @brief The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
      double escape_reset_dist_, escape_reset_theta_; ///< @brief The distance the robot must travel before it can leave escape mode
      bool holonomic_robot_; ///< @brief Is the robot holonomic or not?

      double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_; ///< @brief Velocity limits for the controller

      double backup_vel_; ///< @brief The velocity to use while backing up

      bool dwa_;  ///< @brief Should we use the dynamic window approach?
      bool heading_scoring_; ///< @brief Should we score based on the rollout approach or the heading approach
      double heading_scoring_timestep_; ///< @brief How far to look ahead in time when we score a heading
      bool simple_attractor_;  ///< @brief Enables simple attraction to a goal point

      std::vector<double> y_vels_; ///< @brief Y velocities to explore

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double sim_period_; ///< @brief The number of seconds to use to compute max/min vels for dwa

      double inscribed_radius_, circumscribed_radius_;

      std::mutex configuration_mutex_;
      // std::lock_guard<boost::mutex_t> configuration_mutex_;

      /**
       * @brief  Compute x position based on velocity
       * @param  xi The current x position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new x position
       */
      inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
        return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute y position based on velocity
       * @param  yi The current y position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new y position
       */
      inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
        return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute orientation based on velocity
       * @param  thetai The current orientation
       * @param  vth The current theta velocity
       * @param  dt The timestep to take
       * @return The new orientation
       */
      inline double computeNewThetaPosition(double thetai, double vth, double dt){
        return thetai + vth * dt;
      }

      //compute velocity based on acceleration
      /**
       * @brief  Compute velocity based on acceleration
       * @param vg The desired velocity, what we're accelerating up to
       * @param vi The current velocity
       * @param a_max An acceleration limit
       * @param  dt The timestep to take
       * @return The new velocity
       */
      inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
        if((vg - vi) >= 0) {
          return std::min(vg, vi + a_max * dt);
        }
        return std::max(vg, vi - a_max * dt);
      }

      void getMaxSpeedToStopInTime(double time, double& vx, double& vy, double& vth){
        vx = acc_lim_x_ * std::max(time, 0.0);
        vy = acc_lim_y_ * std::max(time, 0.0);
        vth = acc_lim_theta_ * std::max(time, 0.0);
      }

      double lineCost(int x0, int x1, int y0, int y1);
      double pointCost(int x, int y);
      double headingDiff(int cell_x, int cell_y, double x, double y, double heading);
  };
};

#endif
