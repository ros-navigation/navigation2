# Description of an exclusion zone for the collision monitor / detector.
# Used by the AddExclusionZone service to add zones at runtime.

string zone_name                                    # Unique name within the source
string type                                         # "polygon" or "circle"
string frame_id                                     # TF frame the zone is anchored to (empty = base_frame_id)
geometry_msgs/Point32[] points                      # Polygon vertices in frame_id (ignored for circle)
float64 radius 0.0                                  # Circle radius in metres (ignored for polygon)
float64 min_height -1.7976931348623158e+308          # Lower z-bound in base frame (default: -DBL_MAX)
float64 max_height 1.7976931348623158e+308           # Upper z-bound in base frame (default: +DBL_MAX)
bool enabled true                                   # Whether zone is active on creation
bool visualize false                                # Whether to publish the zone polygon for rviz
float64 frame_hold_timeout 0.0                      # Extra staleness allowance (seconds)
