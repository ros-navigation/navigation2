cmake_minimum_required(VERSION 2.8.3)
project(nav_2d_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(FILES
    Path2D.msg
    Pose2D32.msg
    Pose2DStamped.msg
    Twist2D.msg
    Twist2D32.msg
    Twist2DStamped.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
