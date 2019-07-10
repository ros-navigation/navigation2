# Generic set of parameters to use with base local planners
# To use:
#
#  from local_planner_limits import add_generic_localplanner_params
#  gen = ParameterGenerator()
#  add_generic_localplanner_params(gen)
#  ...
#
# Using these standard parameters instead of your own allows easier switching of local planners

# need this only for dataype declarations
from dynamic_reconfigure.parameter_generator_catkin import double_t, bool_t


def add_generic_localplanner_params(gen):
    # velocities
    gen.add("max_vel_trans", double_t, 0, "The absolute value of the maximum translational velocity for the robot in m/s", 0.55, 0)
    gen.add("min_vel_trans", double_t, 0, "The absolute value of the minimum translational velocity for the robot in m/s", 0.1, 0)

    gen.add("max_vel_x", double_t, 0, "The maximum x velocity for the robot in m/s", 0.55)
    gen.add("min_vel_x", double_t, 0, "The minimum x velocity for the robot in m/s", 0.0)

    gen.add("max_vel_y", double_t, 0, "The maximum y velocity for the robot in m/s", 0.1)
    gen.add("min_vel_y", double_t, 0, "The minimum y velocity for the robot in m/s", -0.1)

    gen.add("max_vel_theta", double_t, 0, "The absolute value of the maximum rotational velocity for the robot in rad/s",  1.0, 0)
    gen.add("min_vel_theta", double_t, 0, "The absolute value of the minimum rotational velocity for the robot in rad/s", 0.4, 0)

    # acceleration
    gen.add("acc_lim_x", double_t, 0, "The acceleration limit of the robot in the x direction", 2.5, 0, 20.0)
    gen.add("acc_lim_y", double_t, 0, "The acceleration limit of the robot in the y direction", 2.5, 0, 20.0)
    gen.add("acc_lim_theta", double_t, 0, "The acceleration limit of the robot in the theta direction", 3.2, 0, 20.0)
    gen.add("acc_lim_trans", double_t, 0, "The absolute value of the maximum translational acceleration for the robot in m/s^2", 0.1, 0)

    gen.add("prune_plan", bool_t, 0, "Start following closest point of global plan, not first point (if different).", False)

    gen.add("xy_goal_tolerance", double_t, 0, "Within what maximum distance we consider the robot to be in goal", 0.1)
    gen.add("yaw_goal_tolerance", double_t, 0, "Within what maximum angle difference we consider the robot to face goal direction", 0.1)

    gen.add("trans_stopped_vel", double_t, 0, "Below what maximum velocity we consider the robot to be stopped in translation", 0.1)
    gen.add("theta_stopped_vel", double_t, 0, "Below what maximum rotation velocity we consider the robot to be stopped in rotation", 0.1)
