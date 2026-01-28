# Real-time tracking error for Nav2 controller

std_msgs/Header header
# The sign of the position tracking error indicates which side of the path the robot is on
# Positive sign indicates the robot is to the left of the path, negative to the right
float32 position_tracking_error
# The sign of the heading tracking error indicates the angular difference between
# the robot's heading and the path.
# Positive sign means the robot heading is rotated to the right of the path direction,
# negative sign means it is rotated to the left.
float32 heading_tracking_error
uint32 current_path_index
geometry_msgs/PoseStamped robot_pose
float32 distance_to_goal
float32 speed
float32 remaining_path_length
