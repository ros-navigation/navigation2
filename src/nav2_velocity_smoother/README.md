# Velocity Smoother

The ``nav2_velocity_smoother`` is a package containing a lifecycle-component node for smoothing velocities sent by Nav2 to robot controllers.
The aim of this package is to implement velocity, acceleration, and deadband smoothing from Nav2 to reduce wear-and-tear on robot motors and hardware controllers by smoothing out the accelerations/jerky movements that might be present with some local trajectory planners' control efforts.

It supports differential drive and omnidirectional robot platforms primarily, but is applicable to ackermann as well with some interpretations of ``Twist``. It was built by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research](https://www.sra.samsung.com/). 

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html) for additional parameter descriptions.

## Features

This package was created to do the following:

- Limit velocity commands by kinematic constraints, including velocity and acceleration
- Limit velocities based on deadband regions
- Stop sending velocities after a given timeout duration of no new commands (due to stopped navigation)
- Send a zero-velocity command at velocity timeout to stop the robot, in case not properly handled
- Support Omni and differential drive robots (e.g. X, Y, Theta)
- Smooth velocities proportionally in the same direction as commanded, whenever possible within kinematic limits
- Provide open loop and closed loop options
- Component nodes for use in single-process systems and stand-alone node format
- Dynamically reconfigurable parameters

## Design

This is a lifecycle-component node, using the lifecycle manager for state management and composition for process management.
It is designed to take in a command from Nav2's controller server and smooth it for use on robot hardware controllers.
Thusly, it takes in a command via the `cmd_vel` topic and produces a smoothed output on `smoothed_cmd_vel`.

The node is designed on a regular timer running at a configurable rate.
This is in contrast to simply computing a smoothed velocity command in the callback of each `cmd_vel` input from Nav2.
This allows us to interpolate commands at a higher frequency than Nav2's local trajectory planners can provide.
For example, if a local trajectory planner is running at 20hz, the velocity smoother can run at 100hz to provide approximately 5 messages to a robot controller which will be smoothed by kinematic limits at each timestep.
This provides a more regular stream of commands to a robot base and interpolates commands between the current velocity and the desired velocity more finely for smoother acceleration / motion profiles.
While this is not required, it is a nice design feature.
It is possible to also simply run the smoother at `cmd_vel` rate to smooth velocities alone without interpolation.

There are two primary operation modes: open and closed loop.
In open-loop, the node assumes that the robot was able to achieve the velocity send to it in the last command which was smoothed (which should be a good assumption if acceleration limits are set properly).
This is useful when robot odometry is not particularly accurate or has significant latency relative to `smoothing_frequency` so there isn't a delay in the feedback loop.
In closed-loop, the node will read from the odometry topic and apply a smoother over it to obtain the robot's current speed.
This will be used to determine the robot's current velocity and therefore achievable velocity targets by the velocity, acceleration, and deadband constraints using live data.

## Parameters

See inline description of parameters in the `VelocitySmoother`.

```
velocity_smoother:
  ros__parameters:
  	smoothing_frequency: 20.0  # Rate to run smoother
  	scale_velocities: false  # scale velocities proportionally if any axis is outside of acceleration range to follow same vector, if possible
  	feedback: "OPEN_LOOP"  # Type of feedback for current speed. Open loop uses the last smoothed output. Closed loop uses robot odometry
  	max_velocity: [0.5, 0.0, 2.5]  # Maximum velocities, ordered [Vx, Vy, Vw]
   	min_velocity: [-0.5, 0.0, -2.5]  # Minimum velocities, ordered [Vx, Vy, Vw]
   	deadband_velocity: [0.0, 0.0, 0.0]  # A deadband of velocities below which they should be zero-ed out for sending to the robot base controller, ordered [Vx, Vy, Vw]
   	velocity_timeout: 1.0  # Time (s) after which if no new velocity commands are received to zero out and stop
   	max_accel: [2.5, 0.0, 3.2]  # Maximum acceleration, ordered [Ax, Ay, Aw]
   	max_decel: [-2.5, 0.0, -3.2]  # Maximum deceleration, ordered [Ax, Ay, Aw]
   	odom_topic: "odom"  # Topic of odometry to use for estimating current velocities
   	odom_duration: 0.1  # Period of time (s) to sample odometry information in for velocity estimation
	enable_stamped_cmd_vel: false # Whether to stamp the velocity. True uses TwistStamped. False uses Twist
```

## Topics

| Topic            | Type                    | Use                           |
|------------------|-------------------------|-------------------------------|
| smoothed_cmd_vel | geometry_msgs/Twist or  geometry_msgs/TwistStamped | Publish smoothed velocities   |
| cmd_vel          | geometry_msgs/Twist or  geometry_msgs/TwistStamped | Subscribe to input velocities |


## Install

```
sudo apt-get install ros-<ros2-distro>-nav2-velocity-smoother
```

## Etc (Important Side Notes)

Typically: if you have low rate odometry, you should use open-loop mode or set the smoothing frequency relatively similar to that of your `cmd_vel` topic. If you have high rate odometry, you can use closed-loop mode with a higher smoothing frequency since you'll have more up to date information to smooth based off of. Do not exceed the smoothing frequency to your odometry frequency in closed loop mode. Also consider the ``odom_duration`` to use relative to your odometry publication rate and noise characteristics.
If the smoothing frequency out paces odometry or poorly selected ``odom_duration``s are used, the robot can oscillate and/or accelerate slowly due to latency in closed loop mode.

When in doubt, open-loop is a reasonable choice for most users.

The minimum and maximum velocities for rotation (e.g. ``Vw``) represent left and right turns. While we make it possible to specify these separately, most users would be wise to set these values the same (but signed) for rotation. Additionally, the parameters are signed, so it is important to specify maximum deceleration with negative signs to represent deceleration. Minimum velocities with negatives when moving backward, so backward movement can be restricted by setting this to ``0``.

Deadband velocities are minimum thresholds, below which we set its value to `0`. This can be useful when your robot's breaking torque from stand still is non-trivial so sending very small values will pull high amounts of current.

The `VelocitySmoother` node makes use of a [nav2_util::TwistSubscriber](../nav2_util/README.md#twist-publisher-and-twist-subscriber-for-commanded-velocities).