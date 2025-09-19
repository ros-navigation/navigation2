cmake_minimum_required(VERSION 3.8)
project(nav_2d_msgs)

find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(nav_2d_msgs
    "msg/Pose2D32.msg"
    "msg/Twist2D.msg"
    "msg/Twist2D32.msg"
    "msg/Twist2DStamped.msg"
    DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
