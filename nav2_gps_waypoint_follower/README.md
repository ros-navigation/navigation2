# Nav2 GPS Waypoint Follower

This package is analogous to `nav2_waypoint_follower`, `nav2_gps_waypoint_follower` provides an action server interface that accepts GPS waypoint following requests by using tools provided by `robot_localization` and `nav2_waypoint_follower`.  The action msg has following properties; 

```yaml
#goal definition
sensor_msgs/NavSatFix[] waypoints
---
#result definition
int32[] missed_waypoints
---
#feedback
uint32 current_waypoint
```

A use case can read a set of GPS waypoints from a YAML file an create a client to action server named as `FollowGPSWaypoints`.  
For instance;

```cpp
using ClientT = nav2_msgs::action::FollowGPSWaypoints;
rclcpp_action::Client<ClientT>::SharedPtr gps_waypoint_follower_action_client_;
gps_waypoint_follower_action_client_ = rclcpp_action::create_client<ClientT>(this, "FollowGPSWaypoints");

```

All other functionalities provided by `nav2_waypoint_follower` such as WaypointTaskExecutors are usable and can be configured in WaypointTaskExecutor.  