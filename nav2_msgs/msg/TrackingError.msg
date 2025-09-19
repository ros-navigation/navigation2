# Real-time tracking error for Nav2 controller

std_msgs/Header header
# The sign of the tracking error indicates which side of the path the robot is on
# Positive sign indicates the robot is to the left of the path, negative to the right
float32 tracking_error
uint32 current_path_index
geometry_msgs/PoseStamped robot_pose
float32 distance_to_goal
float32 speed