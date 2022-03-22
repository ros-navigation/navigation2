# Model Predictive Path Integral Controller

## Differential drive  
![](.resources/demo-diff.gif)

## Omni
![](.resources/demo-omni.gif)

## Overview

Navigation2 Controller plugin. Currently testing on ros2 foxy.

This is a controller (local trajectory planner) that implements model predictive 
path integral control to track a path with collision avoidance. 

The main idea of the algorithm is to sample batch of control sequences with specified time step for each control, 
Having inital state of robot (pose, velocity) and batch of controls use iteratively "model" to predict real velocities for each time step in batch.

this can be explained as follows V(t+1) = M(t), where 

  - V(t+1) - predicted velocities of batch at time step t + 1
  - M(T) - Function that predicts real velocities at t + 1 step, by given velocities and control actions at t step.

Then velocities integrated to get trajectories. For each trajectory, the cost function is calculated. 
All control sequences are weighted by trajectories costs using softmax function to get final control sequence.

## Dependencies 

This uses the usual ROS tools for dependency management, so please use ``rosdep`` to install the dependencies. 

Note: If running on Ubuntu 20.04 or other OS's that `xtensor` is not released in in binary form, please manually install `xtensor` v 0.24.0 and `xtl` v 0.7.0. These are simply headers so the install process is trivially short, unfortunately the `xtensor` project isn't available in package managers in some common-place operating systems (albeit, all necessary ROS OS versions) so you may be required to do this yourself if building from source.

```
git clone git@github.com:xtensor-stack/xtensor.git -b 0.24.0
cd xtensor
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make install

git clone git@github.com:xtensor-stack/xtl.git -b 0.7.0
cd xtl
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make install
```


## Configuration

### Controller params
 | Parameter             | Type   | Definition                                                                                                                                                                                                                                                        |
 | --------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 | iteration_count       | int    | Iteration count in MPPI algorithm                                                                                                                                                                                                                                 |
 | max_robot_pose_search_dist | double | Upper bound on integrated distance along the global plan to search for the closest pose to the robot pose. This should be left as the default unless there are paths with loops and intersections that do not leave the local costmap, in which case making this value smaller is necessary to prevent shortcutting. |
 | transform_tolerance   | double | TF tolerance to transform poses                                                                                                                                                                                                                                   |
 | batch_size            | int    | Count of randomly sampled trajectories                                                                                                                                                                                                                            |
 | time_steps            | int    | Number of time steps (points) in each sampled trajectory                                                                                                                                                                                                          |
 | model_dt              | double | Time interval between two sampled points in trajectories                                                                                                                                                                                                          |
 | vx_std                | double | Sampling standart deviation for VX                                                                                                                                                                                                                                |
 | vy_std                | double | Sampling standart deviation for VY                                                                                                                                                                                                                                |
 | wx_std                | double | Sampling standart deviation for WX                                                                                                                                                                                                                                |
 | vx_max                | double | Max VX                                                                                                                                                                                                                                                            |
 | vy_max                | double | Max VY                                                                                                                                                                                                                                                            |
 | wz_max                | double | Max WZ                                                                                                                                                                                                                                                            |
 | temperature           | double | Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in considiration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories withou cost consideration |
 | visualize             | bool   | Use visualization                                                                                                                                                                                                                                                 |
 | motion_model          | string | Type of model [DiffDrive, Omni, Ackermann]                                                                                                                                                                                                                               |

#### CriticScorer params

 | Parameter       | Type        | Definition                                                                                                  |
 | --------------- | ----------- | ----------------------------------------------------------------------------------------------------------- |
 | critics         | string list | Critics (plugins) names

#### GoalCritic params
 | Parameter       | Type   | Definition                                                                                                  |
 | --------------- | ------ | ----------------------------------------------------------------------------------------------------------- |
 | goal_weight     | double |                                                                                                             |
 | goal_power      | int    |                                                                                                             |

#### GoalAngleCritic params
 | Parameter                        | Type   | Definition                                                                                                  |
 | ---------------                  | ------ | ----------------------------------------------------------------------------------------------------------- |
 | goal_angle_cost_weight           | double |                                                                                                             |
 | goal_angle_cost_power            | int    |                                                                                                             |
 | threshold_to_consider_goal_angle | double | Minimal distance between robot and goal above which angle goal cost considered                              |

#### AngleToGoalCritic params
 | Parameter                 | Type   | Definition                                                                                                  |
 | ---------------           | ------ | ----------------------------------------------------------------------------------------------------------- |
 | angle_to_goal_cost_weight | double |                                                                                                             |
 | angle_to_goal_cost_power  | int    |                                                                                                             |

#### [Approx]ReferenceTrajectoryCritic params
 | Parameter             | Type   | Definition                                                                                                  |
 | ---------------       | ------ | ----------------------------------------------------------------------------------------------------------- |
 | reference_cost_weight | double |                                                                                                             |
 | reference_cost_power  | int    |                                                                                                             |

#### ObstaclesCritic params
 | Parameter                     | Type   | Definition                                                                                                  |
 | ---------------               | ------ | ----------------------------------------------------------------------------------------------------------- |
 | consider_footprint            | bool   |                                                                                                             |
 | obstacle_cost_weight          | double |                                                                                                             |
 | obstacle_cost_power           | int    |                                                                                                             |
 | inflation_cost_scaling_factor | int    | Must be set accurately according to inflation layer params                                                  |
 | inflation_radius              | double | Must be set accurately according to inflation layer params                                                  |



### XML configuration example
```
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "mppi::Controller"
      time_steps: 15
      model_dt: 0.1
      batch_size: 400
      vx_std: 0.1
      vy_std: 0.1
      wz_std: 0.6
      vx_max: 0.5
      vy_max: 0.5
      wz_max: 1.3
      iteration_count: 2
      temperature: 0.25
      motion_model: "DiffDrive"
      visualize: false
      critics: [ "GoalCritic", "GoalAngleCritic", "PathAngleCritic", "ReferenceTrajectoryCritic", "ObstaclesCritic" ]
      GoalCritic:
        goal_cost_power: 1
        goal_cost_weight: 8.0
      GoalAngleCritic:
        goal_angle_cost_power: 1
        goal_angle_cost_weight: 15.0
        threshold_to_consider_goal_angle: 0.20
      ReferenceTrajectoryCritic:
        reference_cost_power: 1
        reference_cost_weight: 5.0
      ObstaclesCritic:
        consider_footprint: true
        obstacle_cost_power: 1
        obstacle_cost_weight: 0.5
      PathAngleCritic:
        path_angle_cost_power: 1
        path_angle_cost_weight: 0.5
```

## Topics

| Topic                     | Type                             | Description                                                           |
|---------------------------|----------------------------------|-----------------------------------------------------------------------|
| `trajectories`            | `visualization_msgs/MarkerArray` | Randomly generated trajectories, including resulting control sequence |
| `transformed_global_plan` | `nav_msgs/Path`                  | Part of global plan considered by local planner                       |

## References
[AutoRally](https://github.com/AutoRally/autorally)
