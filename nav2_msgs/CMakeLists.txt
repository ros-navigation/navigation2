cmake_minimum_required(VERSION 3.5)
project(nav2_msgs)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geographic_msgs)
find_package(action_msgs REQUIRED)

nav2_package()

# TODO(jwallace42): This is a work around for https://github.com/ros2/rosidl_typesupport_fastrtps/issues/28
add_compile_options(-Wno-error=deprecated)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CollisionMonitorState.msg"
  "msg/CollisionDetectorState.msg"
  "msg/Costmap.msg"
  "msg/CostmapMetaData.msg"
  "msg/CostmapUpdate.msg"
  "msg/CostmapFilterInfo.msg"
  "msg/SpeedLimit.msg"
  "msg/VoxelGrid.msg"
  "msg/BehaviorTreeStatusChange.msg"
  "msg/BehaviorTreeLog.msg"
  "msg/Particle.msg"
  "msg/ParticleCloud.msg"
  "msg/MissedWaypoint.msg"
  "msg/Route.msg"
  "msg/RouteNode.msg"
  "msg/RouteEdge.msg"
  "msg/EdgeCost.msg"
  "srv/GetCost.srv"
  "srv/GetCostmap.srv"
  "srv/IsPathValid.srv"
  "srv/ClearCostmapExceptRegion.srv"
  "srv/ClearCostmapAroundRobot.srv"
  "srv/ClearCostmapAroundPose.srv"
  "srv/ClearEntireCostmap.srv"
  "srv/ManageLifecycleNodes.srv"
  "srv/LoadMap.srv"
  "srv/SaveMap.srv"
  "srv/SetInitialPose.srv"
  "srv/ReloadDockDatabase.srv"
  "srv/SetRouteGraph.srv"
  "srv/DynamicEdges.srv"
  "srv/Toggle.srv"
  "action/AssistedTeleop.action"
  "action/BackUp.action"
  "action/ComputePathToPose.action"
  "action/ComputePathThroughPoses.action"
  "action/ComputeRoute.action"
  "action/ComputeAndTrackRoute.action"
  "action/DriveOnHeading.action"
  "action/SmoothPath.action"
  "action/FollowPath.action"
  "action/NavigateToPose.action"
  "action/NavigateThroughPoses.action"
  "action/Wait.action"
  "action/Spin.action"
  "action/DummyBehavior.action"
  "action/FollowWaypoints.action"
  "action/FollowGPSWaypoints.action"
  "action/DockRobot.action"
  "action/UndockRobot.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
