

TODO

- homologate find_packages/dependencies/package.xml with what's used
- reset ruckig if paused between navigation requests


Requirements:
	- OK Velocity limits
	- OK Acceleration limits
	- OK Deadband velocities (don't command something below a number)
	- OK If no command after X time, send 0 -- safety timeout
	- Support for X/Y for omni
	- Jerk limits?
	- Commanded vs measured odometry velocities option
	- tests


Design:
	- OK Run on an independent timer
	- OK Take in cmd_vel, use last
	- This way can interpolate if running at a frequency much higher than controller
	- Component node + in launch files
	- dynamic parameters
	- 