cmake_minimum_required(VERSION 2.8.3)
project(dwb_msgs)

find_package(catkin REQUIRED
    COMPONENTS geometry_msgs nav_2d_msgs message_generation nav_msgs
)

add_message_files(FILES
    CriticScore.msg
    LocalPlanEvaluation.msg
    Trajectory2D.msg
    TrajectoryScore.msg
)
add_service_files(FILES
    DebugLocalPlan.srv
    GenerateTrajectory.srv
    GenerateTwists.srv
    GetCriticScore.srv
    ScoreTrajectory.srv
)
generate_messages(DEPENDENCIES geometry_msgs nav_2d_msgs nav_msgs)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime nav_2d_msgs nav_msgs
)
