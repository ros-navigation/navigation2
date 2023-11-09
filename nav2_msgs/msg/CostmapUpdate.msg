# Update msg for Costmap containing the modified part of Costmap
std_msgs/Header header

uint32 x
uint32 y

uint32 size_x
uint32 size_y

# The cost data, in row-major order, starting with (x,y) from 0-255 in Costmap format rather than OccupancyGrid 0-100.
uint8[] data
