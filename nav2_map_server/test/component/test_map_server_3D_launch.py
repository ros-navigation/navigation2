
import os
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    launchFile = os.path.join(os.getenv('TEST_LAUNCH_DIR'), 'map_server_3D_node.launch.py')
    testExecutable = os.getenv('TEST_EXECUTABLE')
    ld = LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([launchFile])),
    ])
    test1_action = ExecuteProcess(
        cmd=[testExecutable],
        name='test_map_server_3D_node',
    )
    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    os.chdir(os.getenv('TEST_LAUNCH_DIR'))
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
