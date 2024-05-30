# This hold basic information about the characteristics of the Costmap

# The time at which the static map was loaded
builtin_interfaces/Time map_load_time

# The time of the last update to costmap
builtin_interfaces/Time update_time

# The corresponding layer name
string layer

# The map resolution [m/cell]
float32 resolution

# Number of cells in the horizontal direction
uint32 size_x

# Number of cells in the vertical direction
uint32 size_y

# The origin of the costmap [m, m, rad].
# This is the real-world pose of the cell (0,0) in the map.
geometry_msgs/Pose origin
