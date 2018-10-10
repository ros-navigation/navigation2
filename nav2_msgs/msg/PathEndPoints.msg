std_msgs/Header header

# The start pose for the plan
geometry_msgs/Pose start

# The final pose of the goal
geometry_msgs/Pose goal

# If the goal is obstructed, how many meters the planner can
# relax the constraint in x and y before failing.
float32 tolerance