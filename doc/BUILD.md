Build Instructions
==================

## 1. Platform
* Ubuntu 18.04

## 2. Manual Build Steps
* The instructions are to build **Navigation2** package and its dependency packages.
* To build **Navigation2** package from script, kindly refer [QuickStart](BUILD.md#quickstart-using-initial-setup-script).
* To build **Navigation2** package for `main` branch, select the latest ROS2 distro in place of `ros2-distro-name` during `rosdep install`.
* To build **Navigation2** package for specific distro branch (e.g- dashing-devel), select the matching ROS2 distro in place of `ros2-distro-name` during `rosdep install`.


### 2.1 ROS2
* Install ROS2 with dependencies from the ROS2 Installation Page: https://index.ros.org/doc/ros2/Installation/
* Install the latest distribution of ROS2 to support Navigation2 `main`.
* ROS2 can be installed either from binary packages or build from source. Choose to build ROS2 from source to support Navigation2 `master`.

### 2.2 Navigation2 Dependencies
If ROS2 installed from source (Ignore if installed from binary packages)
```console
$ source ~/ros2_ws/install/setup.bash
```
Fetch, build and install navigation2 dependencies:
```console
$ mkdir -p ~/ros2_nav_dependencies_ws/src
$ cd ~/ros2_nav_dependencies_ws
$ wget https://raw.githubusercontent.com/ros-planning/navigation2/eloquent-devel/tools/ros2_dependencies.repos
$ vcs import src < ros2_dependencies.repos
$ rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro-name>
e.g- ros2-distro-name = dashing
$ colcon build --symlink-install
```

### 2.3 Navigation2

Fetch, build and install navigation2 stack:
```console
$ source ~/ros2_nav_dependencies_ws/install/setup.bash
$ mkdir -p ~/navigation2_ws/src
$ cd ~/navigation2_ws/src
$ git clone https://github.com/ros-planning/navigation2.git -b <nav2-branch-name>
e.g- nav2-branch-name = master
$ cd ~/navigation2_ws
$ rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro-name>
e.g- ros2-distro-name = dashing
$ colcon build --symlink-install
```
### 2.4 Turtlebot3
Turtlebot3 is one of the target robot platform to run navigation2. Ignore building Turtlebot3 packages if turtlebot3 is not your target.

If ROS2 installed from source (Ignore if installed from binary packages)
```console
$ source ~/ros2_ws/install/setup.bash
```
Fetch, build and install turtlebot3 packages:
```console
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/eloquent-devel/turtlebot3.repos
$ vcs import src < turtlebot3.repos
$ rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro-name>
e.g- ros2-distro-name = dashing
$ colcon build --symlink-install
```

### 2.4 Quick test
$ source ~/navigation2_ws/install/setup.bash
$ source ~/turtlebot3_ws/install/setup.bash
$ export TURTLEBOT3_MODEL=waffle
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
$ ros2 launch nav2_bringup nav2_tb3_simulation_launch.py 

## 3. Conclusion
After installation of **Navigation2** and required dependencies, test **Navigation2** by following steps in [nav2_bringup README](../nav2_bringup/bringup/README.md)

## 4. Reporting Issue
If run into any issue, feel free to submit pull request or report issue in this project.

Quickstart using initial setup script
----------

### Steps
- Install all ROS2 dependencies from the ROS2 Installation page: https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/

- Ensure there are no ROS environment variables set in your terminal or `.bashrc` file before taking the steps below.*

```sh
mkdir <directory_for_workspaces>
cd <directory_for_workspaces>
wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/initial_ros_setup.sh
chmod a+x initial_ros_setup.sh
./initial_ros_setup.sh
```

### Summary of what's being done

The `initial_ros_setup.sh` script downloads four ROS workspaces and then builds them in the correct order. The four workspaces are:

 * ROS 2 release - This is the ROS 2 latest release as defined by the repos file found on [https://github.com/ros2/ros2/]

 * ROS 2 Dependencies - This is a set of ROS 2 packages that aren't included in the ROS 2 release yet, but which the navstack depends on. This also includes packages which are part of the ROS 2 release where we need a different version.

 * Navigation 2 - This repo.

 After all the workspaces are downloaded, the `navigation2/tools/build_all.sh` script is run which builds each repo in the order listed above using the `colcon build --symlink-install` command.

### Options

The `initial_ros_setup.sh` accepts the following options:
 * `--no-ros2` This skips downloading and building the ROS 2 release. Instead it uses the binary packages and setup.sh installed in `/opt/ros/bouncy`
 * `--download-only` This skips the build steps

Use Case Recommendations
----------

### Developer

For developers, running the `initial_ros_setup.sh` once makes sense. After that, you'll typically want to maintain each repo manually using git.

Most work will be done in the `navigation2` workspace, so just building that will save time.

To build just `navigation2`,
```sh
cd <directory_for_workspaces>/navigation2_ws
source ../ros2_nav_dependencies_ws/install/setup.sh
colcon build --symlink-install
```

In the case that the developer changes any dependencies, they can run
`<directory_for_workspaces>/navigation2_ws/src/navigation2/tools/build_all.sh` in a clean environment to get everything rebuilt easily

### Build System

An automated build system could make a clean directory and run the `initial_ros_setup.sh` script each time, however that will generate a lot of unneccessary load on the upstream repo servers, and result in very long builds.

Instead, it would be better to do an initial download of all the source and dependencies
```sh
./initial_ros_setup.sh --download-only
```

Then the CI tool can monitor the `navigation2` repo, update it as necessary, and rebuild using either the `<directory_for_workspaces>/navigation2_ws/src/navigation2/tools/build_all.sh` script or by running
```sh
cd <directory_for_workspaces>/navigation2_ws/src/navigation2
source ../ros2_nav_dependencies_ws/install/setup.sh
colcon build --symlink-install
```
