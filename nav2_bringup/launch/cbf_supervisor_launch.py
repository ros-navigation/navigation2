from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
import os

def generate_launch_description():
    launchDesc = LaunchDescription()
    # If the Supervisor package is available on system (debian install)
    # then include the supervisor launch file
    pkg = get_package_share_directory("lll_supervisor")
    launchDesc.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        get_package_share_directory("lll_supervisor"),
                        "launch",
                        "supervisor.launch.py",
                    ]
                )
            )
        )
    )

    return launchDesc
