# Full Scoring for running the local planner

# Header, used for timestamp
std_msgs/Header header
# All the trajectories evaluated and their scores
TrajectoryScore[] twists
# Convenience index of the best (lowest) score in the twists array
uint16 best_index
# Convenience index of the worst (highest) score in the twists array. Useful for scaling.
uint16 worst_index
