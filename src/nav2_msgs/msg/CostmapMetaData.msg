# This hold basic information about the characterists of the Costmap

# The time at which the static map was loaded
builtin_interfaces/Time map_load_time

# The time of the last update to costmap
builtin_interfaces/Time update_time

# The corresponding layer
string layer

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height

# The origin of the costmap [m, m, rad].
# This is the real-world pose of the cell (0,0) in the map.
geometry_msgs/Pose origin