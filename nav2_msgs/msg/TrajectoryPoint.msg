# Trajectory point state

# Desired time from the trajectory start to arrive at this trajectory sample.
builtin_interfaces/Duration time_from_start

# Pose of the trajectory sample.
geometry_msgs/Pose pose

# Velocity of the trajectory sample.
geometry_msgs/Twist velocity

# Acceleration of the trajectory (optional).
geometry_msgs/Accel acceleration

# Force/Torque to apply at trajectory sample (optional).
geometry_msgs/Wrench effort
