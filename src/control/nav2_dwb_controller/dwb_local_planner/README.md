# dwb_local_planner

 > I suppose it is tempting, if the only tool you have is a hammer, to treat everything as if it were a nail.
 - Abraham Maslow

 > I love/hate pluginlib.
 - David Lu!!

This local planner implementation rewrites and extends the functionality of `dwa_local_planner`, thus it is logically called `dwb_local_planner`. The goal is to make as many parts of the functionality as possible customizable, either through pluginlib or directly extending the implementing classes.

The goal of a local planner is to take the global plan and local costmap and produce the command velocities that will presumably move the robot to the goal. Both dwa and dwb do this via sampling, i.e. generating plausible velocity commands and evaluating them on various metrics and selecting the one with the best score, until the robot reaches its goal.

The form of the evaluation is critical. Let's say we are evaluating a given command to see if it collides with any obstacles in the costmap. The key question is where the robot will drive using the command. For this, you need to know the position and velocity of the robot, plus you will need to consider the kinematics of the robot. To this end, we do not evaluate the velocities in isolation, but instead score the trajectory which contains not only the velocity, but an array of some number of sample poses that we anticipate the robot to drive along.

## Data Structures
The navigation stack is only capable of navigation in 2.5 dimensions (i.e. x, y and theta), thus most of the interfaces deal with `geometry_msgs/Pose2D` and `nav_2d_msgs/Twist2D` rather than the more general `Pose` and `Twist`.

## Code Structure
### Twist Generator
The first component is the twist generator, which is responsible for determining which velocity commands are evaluating and creating trajectories from them. The interface for generating the commands is iterator-based.
```
void startNewIteration(const nav_2d_msgs::Twist2D& current_velocity)
bool hasMoreTwists()
bool nextTwist(nav_2d_msgs::Twist2D& twist)
```
For debugging purposes, there is also a ROS service interface for the twist generator.
```
# dwb_msgs/srv/GenerateTwists.srv
nav_2d_msgs/Twist2D current_vel
---
nav_2d_msgs/Twist2D[] twists
```

The second half is for generating the trajectory, which creates a Trajectory2D.
```
# dwb_msgs/msg/Trajectory2D.msg
nav_2d_msgs/Twist2D velocity
duration duration
geometry_msgs/Pose2D[] poses
```

Code API:
```
bool generateTrajectory(const geometry_msgs::Pose2D& start_pose, const nav_2d_msgs::Twist2D& start_vel,
                        const nav_2d_msgs::Twist2D& cmd_vel,
                        dwb_msgs::Trajectory2D& traj)
```

This also has a ROS service interface.
```
# dwb_msgs/srv/GenerateTrajectory.srv
geometry_msgs/Pose2D start_pose
nav_2d_msgs/Twist2D start_vel
nav_2d_msgs/Twist2D cmd_vel
---
dwb_msgs/Trajectory2D traj
```

How precisely DWB performs these tasks is relegated to a plugin.

## Goal Checker
Are we there yet? Another plugin determines whether the robot has reached its goal. This allows for variation in combining how accurate the xy position has to be with the rotation, and whether the robot has stopped completely.

Code API:
```
virtual bool isGoalReached(const geometry_msgs::Pose2D& query_pose, const geometry_msgs::Pose2D& goal_pose,
                           const nav_2d_msgs::Twist2D& velocity)
```

## Trajectory Critics
[Critics](https://www.youtube.com/watch?v=X6I_dKUYyI4) like to give things scores. Once we know we're not a the goal and have a bunch of candidate trajectories, we evaluate them based on a collections of TrajectoryCritics. Here's the life-cycle.

 * `void initialize(std::string name, std::string parent_namespace, costmap_2d::Costmap2DROS* costmap_ros)` - called once on startup, and then calls `onInit`
 * `void onInit()` - May be overwritten to load parameters as needed.
 * `void reset()` - called at the beginning of every new navigation, i.e. when we get a new global plan.
 * `bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel, const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan)` - called once per iteration of the planner, prior to the evaluation of all the trajectories
 * `double scoreTrajectory(const dwb_msgs::Trajectory2D& traj)` - called once per trajectory
 * `void debrief(const nav_2d_msgs::Twist2D& cmd_vel)` - called after all the trajectories to notify what trajectory was chosen.

Each critic will provide a `double` score and has an associated scale. The score used for the trajectory as a whole will be the sum of all the critic scores multiplied by their respective scales.

There is also a ROS service interface associated with the scoring trajectories.
```
# dwb_msgs/srv/DebugLocalPlan.srv
nav_2d_msgs/Pose2DStamped pose
nav_2d_msgs/Twist2D velocity
nav_2d_msgs/Path2D global_plan
---
LocalPlanEvaluation results
    Header header
    dwb_msgs/TrajectoryScore[] twists
        nav_2d_msgs/SampedTwist2D traj
        dwb_msgs/CriticScore[] scores
            string name
            float32 raw_score
            float32 scale
        float32 total
    uint16 best_index
    uint16 worst_index
```
