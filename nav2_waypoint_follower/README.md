# Nav2 Waypoint Follower

The navigation2 waypoint follower is an example application of how to use the navigation action to complete some sort of orchestrated task. In this example, that task is to take a given set of waypoints and navigate to a set of positions in the order provided in the action request. The last waypoint in the waypoint array is the final position.

The package exposes the `FollowWaypoints` action server of type `nav2_msgs/FollowWaypoints`. It is given an array of waypoints to visit, gives feedback about the current index of waypoint it is processing, and returns a list of waypoints it was unable to complete.

There is a parameterization `stop_on_failure` whether to stop processing the waypoint following action on a single waypoint failure. When false, it will continue onto the next waypoint when the current waypoint fails. The action will exist when either all the waypoint navigation tasks have terminated or when `stop_on_failure`, a single waypoint as failed.
