std_msgs/Header header
# Type of plugin used (keepout filter, speed limit in m/s, speed limit in percent, etc...)
uint8 type
# Name of map filter topic
string map_filter_topic
# Multiplier base offset and multiplier coefficient for conversion of OccGrid.
# Used to convert OccupancyGrid data values to filter space values.
# data -> into some other number space:
# space = data * multiplier + base
float32 base
float32 multiplier
