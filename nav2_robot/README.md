# Nav2 Robot

`nav2_robot` is a package containing an implementation class `Robot` containing information about robot specific parameters, controls, and information. 

Today `Robot` contains basics for interacting with the robot hardware abstraction. It is the entry point for getting information from the odometry measurements and publishing velocity commands.

## ROS1 Comparison

This package does not have a direct counter-part in Navigation. Navigation contains a set of parameter files and reads information from each as needed. This package's intention is to act as a consolidator of robot-specific needs and information for each package across the meta-package to use. An example of another addition to meet that is to have an abstract implementation of a motion model for use in AMCL rather than hard-coding motion models options in the package itself. 

## Future

* This package may soon be deprecated in favor of some robot-specific utilities
* Adding motion model, footprint, base parameters, etc 
