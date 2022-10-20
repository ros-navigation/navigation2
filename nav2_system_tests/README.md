# System Tests

The package provides tests for Component-Testing, Subsystem-Testing, and Full-System integration. Its main goal is to provide a location for smoke and integration tests of the navigation system to ensure that things are working properly on a high level. Unit and specific subsystem testing happens in the packages specific to those algorithms.

Most tests in this package will spin up Gazebo instances of a robot in an environment to have the robot complete some task in the space while tracking a specific modules results.  Some examples include

- System tests of a robot in a sandbox environment trying navigate to pose, navigate through poses, and waypoint following navigation types
- Random planning of thousands of paths in a generated environment to ensure default planners are working properly
- Testing the system can be brought up and down on the lifecycle transitions successfully multiple times
- Testing that the keepout and speed restricted zones work in a practical environment without going into keepout zones and slowing in speed restricted areas
- Testing behaviors in a sandbox environment to ensure they trigger and complete collision checking properly
- Testing system failures are properly recorded and can be recovered from

This is primarily for use in Nav2 CI to establish a high degree of maintainer confidence when merging in large architectural changes to the Nav2 project. However, this is also useful to test installs of Nav2 locally or for additional information.
