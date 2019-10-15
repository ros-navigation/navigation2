# AMCL
Adaptive Monte Carlo Localization (AMCL) is a probabilistic localization module which estimates the position and orientation (i.e. Pose) of a robot in a given known map.

## Overview
Currently, the AMCL module in ROS 2 Navigation System is a direct port from [ROS1 AMCL](http://wiki.ros.org/amcl) package with some minor code re-factoring. The direct port includes all of ROS1 functionalities except running from Bag files.

## Added Feature
AutoLocalization - is implemented by utilizing AMCL's `global_localization` service request and Behavior Tree (BT).  This enables initial pose estimation capability on differential type robot.

 On startup of the navigation stack, the initial robot pose needs to be sent to AMCL otherwise AMCL initializes its filter state to default values with a particle cloud centered around (0,0,0). If the initial pose of the robot is not known and the robot is outside of default particle cloud, particles may not converge when robot moves.

With the AutoLocalization branch of BT, first the `global_localization` service of AMCL is called to randomly disperse all particles through the free space in the map. Once particles are dispersed, the robot rotates, backs up, and, if necessary, rotates again to localize itself. The full implementation is described in the AutoLocalization section of [BT Navigator](https://github.com/ros-planning/navigation2/blob/master/nav2_bt_navigator/README.md)

**Warning**: AutoLocalization actuates robot; currently, obstacle avoidance has not been integrated into this feature. The user is advised to not use this feature on a physical robot for safety reasons.  As of now, this feature should only be used in simulations.

## Future Plan
* Running from Ros bag
* Extending AMCL to work with different type of Sensors
* Improving overall localization performance with fusing data from different sensors such as IMU, Sonar, Radar, Laser, Depth camera, and etc.
