# Navigation Safety Node

The Navigation Safety Node is a safety watchdog node to ensure the robot is acting properly and not about to collide with an obstacle. Typical safety-rated lidars will contain “safety zones” whereas if any sensor points are located in a box around the lidar, then the lidar will send a signal to the robot to stop due to a potential collision.

The node sits below the navigation stack but above the robot controller to do the following:

* Take in the current command velocity from navigation and the most recent laser or RGBD scan.
* Projecting the velocity forward in time N seconds, check if that velocity will result in a collision with any sensor measurements.
* If not, allow the velocity command through to the base.
* If it does collide, scale back the velocity such that the robot will always be at minimum N seconds from a collision.
* Optionally if a flag is set, if M or more points are in defined bounding boxes around the robot, send only 0 commands to enact an emergency stop.


