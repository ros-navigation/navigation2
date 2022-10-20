std_msgs/Header header
# Type of plugin used (keepout filter, speed limit in m/s, speed limit in percent, etc...)
# 0: keepout/lanes filter
# 1: speed limit filter in % of maximum speed
# 2: speed limit filter in absolute values (m/s)
uint8 type
# Name of filter mask topic
string filter_mask_topic
# Multiplier base offset and multiplier coefficient for conversion of OccGrid.
# Used to convert OccupancyGrid data values to filter space values.
# data -> into some other number space:
# space = data * multiplier + base
float32 base
float32 multiplier
