# SMac Planner

The SMacPlanner is a plugin for the Nav2 Planner server.

## Status

This plugin is in development and highly experimental. It can be used but some quirks may exist.

## What is it?

Smac planner is an A* implementation with some unique features. While NavFn will use A* and Dij. for wavefront expansion, it is not itself an A* planner. DLux global planner from David Lu!! at Locus Robotics (for which the naming pattern was followed) cleans up NavFn and the ROS1 navigation global planner. However, it is still itself a wavefront planner with the quirks that that comes with.

We instead have a A* planner that for now operated in 2D. The plan is to expand this into 2.5D and incorperate state validity checking at expansion to allow for rectangular or irregular shaped robots to operate. Also, we will project the motion model rather than operating in a grid structure, but stay tuned. This will resemble Hybrid-A* commonly used on autonomous vehicles with some additional functionality planned including:

- Caching of plans and metrics to use new plan
- Multiresolution queries to save compute for longer range  plans
- Optimizer to smooth plans
