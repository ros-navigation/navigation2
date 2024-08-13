cmake_minimum_required(VERSION 3.5)
project(nav2_msgs)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

nav2_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CollisionMonitorState.msg"
  "msg/Costmap.msg"
  "msg/CostmapMetaData.msg"
  "msg/CostmapFilterInfo.msg"
  "msg/SpeedLimit.msg"
  "msg/VoxelGrid.msg"
  "msg/BehaviorTreeStatusChange.msg"
  "msg/BehaviorTreeLog.msg"
  "msg/Particle.msg"
  "msg/ParticleCloud.msg"
  "srv/GetCostmap.srv"
  "srv/IsPathValid.srv"
  "srv/ClearCostmapExceptRegion.srv"
  "srv/ClearCostmapAroundRobot.srv"
  "srv/ClearEntireCostmap.srv"
  "srv/ManageLifecycleNodes.srv"
  "srv/LoadMap.srv"
  "srv/SaveMap.srv"
  "srv/SetInitialPose.srv"
  "action/AssistedTeleop.action"
  "action/BackUp.action"
  "action/ComputePathToPose.action"
  "action/ComputePathThroughPoses.action"
  "action/DriveOnHeading.action"
  "action/SmoothPath.action"
  "action/FollowPath.action"
  "action/NavigateToPose.action"
  "action/NavigateThroughPoses.action"
  "action/Wait.action"
  "action/Spin.action"
  "action/DummyBehavior.action"
  "action/FollowWaypoints.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
