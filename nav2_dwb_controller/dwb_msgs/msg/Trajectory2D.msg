# For a given velocity command, the poses that the robot will go to in the allotted time.

# Input Velocity
nav_2d_msgs/Twist2D velocity
# Time difference between first and last poses
builtin_interfaces/Duration[] time_offsets
# Poses the robot will go to, given our kinematic model
geometry_msgs/Pose2D[] poses
