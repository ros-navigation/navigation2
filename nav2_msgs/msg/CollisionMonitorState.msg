# Action type for robot in Collision Monitor
uint8 DO_NOTHING=0 # No action
uint8 STOP=1 # Stop the robot
uint8 SLOWDOWN=2 # Slowdown in percentage from current operating speed
uint8 APPROACH=3 # Keep constant time interval before collision
uint8 action_type

# Name of triggered polygon
string polygon_name