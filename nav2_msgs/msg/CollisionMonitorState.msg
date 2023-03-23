# Action type for robot in Collision Monitor
# 0: ACTION_UNKNOWN - Default for initializing action type
# 1: DO_NOTHING - No action
# 2: STOP - Stop the robot
# 3: SLOWDOWN - Slowdown in percentage from current operating speed
# 4: APPROACH - Keep constant time interval before collision
uint8 action_type
# Name of triggered polygon
string polygon_name
