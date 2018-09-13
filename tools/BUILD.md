Build Instructions
==================

Quickstart
----------

### Steps

*Ensure there are no ROS environment variables set in your terminal or `.bashrc` file before taking the steps below.*

```bash
mkdir <workspace_dir>
cd <workspace_dir>
wget https://raw.githubusercontent.com/ros-planning/navigation2/buildscript/tools/initial_ros_setup.bash
chmod a+x initial_ros_setup.bash
./initial_ros_setup.bash
```

### Summary of what's being done

The `initial_ros_setup.bash` script downloads four ROS workspaces and then builds them in the correct order. The four workspaces are:

 * ROS 1 Dependencies - This is a set of ROS 1 repos that are used by the system test. At the time of this writing, it consists solely of Turtlebot 3 packages.

 * ROS 2 release - This is the ROS 2 latest release as defined by the repos file found on [https://github.com/ros2/ros2/]

 * ROS 2 Dependencies - This is a set of ROS 2 packages that aren't included in the ROS 2 release yet, but which the navstack depends on. This also includes packages which are part of the ROS 2 release where we need a different version.

 * Navigation 2 - This repo.

 After all the workspaces are downloaded, the `navigtion2/tools/build_all.sh` script is run which builds each repo in the order listed above using the `colcon build --symlink-install` command (except ROS 1 dependencies which are built using `catkin_make`)

### Options

The `initial_ros_setup.bash` accepts the following options:
 * `--no-ros1` This skips downloading and building the ROS 1 dependencies and skips building the ROS 1 bridge
 * `--download-only` This skips the build steps

Use Case Recommendations
----------

### Developer

For developers, running the `initial_ros_setup.bash` once makes sense. After that, you'll typically want to maintain each repo manually using git.

Most work will be done in the `navigation2` workspace, so just building that will save time.

To build just `navigation2`,
```bash
cd <workspace_dir>/navigation2
source ../navstack_dependencies_ws/install/setup.bash
colcon build --symlink-install
```

In the case that the developer changes any dependencies, they can run
`<workspace_dir>/navigation2/tools/build_all.sh` in a clean environment to get everything rebuilt easily

### Build System

An automated build system could make a clean directory and run the `initial_ros_setup.bash` script each time, however that will generate a lot of unneccessary load on the upstream repo servers, and result in very long builds.

Instead, I'd recommend doing an initial download of all the source and dependencies
```bash
./initial_ros_setup.bash --no-ros1 --download-only
```

Then the CI tool can monitor the `navigation2`, update it as neccessary, and rebuild using either the `<workspace_dir>/navigation2/tools/build_all.sh` script or by running
```bash
cd <workspace_dir>/navigation2
source ../navstack_dependencies_ws/install/setup.bash
colcon build --symlink-install
```
