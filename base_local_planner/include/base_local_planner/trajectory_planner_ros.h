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
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/voxel_grid_model.h>
#include <base_local_planner/trajectory_planner.h>
#include <base_local_planner/map_grid_visualizer.h>

#include <base_local_planner/planar_laser_scan.h>

#include <nav_msgs/msg/Odometry.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_ros/buffer.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

//#include <dynamic_reconfigure/server.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <base_local_planner/odometry_helper_ros.h>

namespace base_local_planner {
  /**
   * @class TrajectoryPlannerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class TrajectoryPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Default constructor for the ros wrapper
       */
      TrajectoryPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      TrajectoryPlannerROS(std::string name,
                           tf2_ros::Buffer* tf,
                           nav2_costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          nav2_costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~TrajectoryPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::msg::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /**
       * @brief  Generate and score a single trajectory
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @param update_map Whether or not to update the map for the planner
       * when computing the legality of the trajectory, this is useful to set
       * to false if you're going to be doing a lot of trajectory checking over
       * a short period of time
       * @return True if the trajectory is legal, false otherwise
       */
      bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true);

      /**
       * @brief  Generate and score a single trajectory
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @param update_map Whether or not to update the map for the planner
       * when computing the legality of the trajectory, this is useful to set
       * to false if you're going to be doing a lot of trajectory checking over
       * a short period of time
       * @return score of trajectory (double)
       */
      double scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true);

      bool isInitialized() {
        return initialized_;
      }

      /** @brief Return the inner TrajectoryPlanner object.  Only valid after initialize(). */
      TrajectoryPlanner* getPlanner() const { return tc_; }

    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level);

      /**
       * @brief Once a goal position is reached... rotate to the goal orientation
       * @param  global_pose The pose of the robot in the global frame
       * @param  robot_vel The velocity of the robot
       * @param  goal_th The desired th value for the goal
       * @param  cmd_vel The velocity commands to be filled
       * @return  True if a valid trajectory was found, false otherwise
       */
      bool rotateToGoal(const geometry_msgs::msg::PoseStamped& global_pose, const geometry_msgs::msg::PoseStamped& robot_vel, double goal_th, geometry_msgs::msg::Twist& cmd_vel);

      /**
       * @brief Stop the robot taking into account acceleration limits
       * @param  global_pose The pose of the robot in the global frame
       * @param  robot_vel The velocity of the robot
       * @param  cmd_vel The velocity commands to be filled
       * @return  True if a valid trajectory was found, false otherwise
       */
      bool stopWithAccLimits(const geometry_msgs::msg::PoseStamped& global_pose, const geometry_msgs::msg::PoseStamped& robot_vel, geometry_msgs::msg::Twist& cmd_vel);

      std::vector<double> loadYVels(ros::NodeHandle node);

      double sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
      }

      WorldModel* world_model_; ///< @brief The world model that the controller will use
      TrajectoryPlanner* tc_; ///< @brief The trajectory controller

      nav2_costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
      nav2_costmap_2d::Costmap2D * costmap_; ///< @brief The costmap the controller will use
      MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function
      tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds
      std::string global_frame_; ///< @brief The frame in which the controller will run
      double max_sensor_range_; ///< @brief Keep track of the effective maximum range of our sensors
      nav_msgs::msg::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
      std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
      double rot_stopped_velocity_, trans_stopped_velocity_;
      double xy_goal_tolerance_, yaw_goal_tolerance_, min_in_place_vel_th_;
      std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
      bool prune_plan_;
      boost::recursive_mutex odom_lock_;

      double max_vel_th_, min_vel_th_;
      double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
      double sim_period_;
      bool rotating_to_goal_;
      bool reached_goal_;
      bool latch_xy_goal_tolerance_, xy_tolerance_latch_;

      ros::Publisher g_plan_pub_, l_plan_pub_;

      dynamic_reconfigure::Server<BaseLocalPlannerConfig> *dsrv_;
      base_local_planner::BaseLocalPlannerConfig default_config_;
      bool setup_;


      bool initialized_;
      base_local_planner::OdometryHelperRos odom_helper_;

      std::vector<geometry_msgs::msg::Point> footprint_spec_;
  };
};
#endif
