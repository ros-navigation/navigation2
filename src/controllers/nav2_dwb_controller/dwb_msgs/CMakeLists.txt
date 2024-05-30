cmake_minimum_required(VERSION 3.5)
project(dwb_msgs)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(dwb_msgs
    "msg/CriticScore.msg"
    "msg/LocalPlanEvaluation.msg"
    "msg/Trajectory2D.msg"
    "msg/TrajectoryScore.msg"
    "srv/DebugLocalPlan.srv"
    "srv/GenerateTrajectory.srv"
    "srv/GenerateTwists.srv"
    "srv/GetCriticScore.srv"
    "srv/ScoreTrajectory.srv"
    DEPENDENCIES geometry_msgs std_msgs nav_2d_msgs nav_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
