# Action type for robot in Collision Monitor
# 0: DO_NOTHING - No action
# 1: STOP - Stop the robot
# 2: SLOWDOWN - Slowdown in percentage from current operating speed
# 3: APPROACH - Keep constant time interval before collision
# 4: ACTION_UNKNOWN - Default for initializing action type
uint8 action_type
# Name of triggered polygon
string polygon_name
