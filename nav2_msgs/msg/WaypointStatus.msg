# Waypoint state for multi-goal navigation or waypoint following
uint8 PENDING=0 # Waypoint is not processed or processing
uint8 COMPLETED=1 # Waypoint is completed
uint8 MISSED=2 # Waypoint is missed
uint8 waypoint_status
uint32 waypoint_index
geometry_msgs/PoseStamped waypoint_pose
uint16 error_code
string error_msg
