# Update msg for Costmap containing the modified part of Costmap
std_msgs/Header header

uint32 x
uint32 y

uint32 size_x
uint32 size_y

# The cost data, in row-major order, starting with (x,y).
int8[] data
