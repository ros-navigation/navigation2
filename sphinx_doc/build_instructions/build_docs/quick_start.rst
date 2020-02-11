.. _quick-start:

Quickstart Using the Initial Setup Script
*****************************************

Steps
=====

  - Install all ROS 2 dependencies from the `ROS2 Installation page <https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup>`_

  - Ensure there are no ROS environment variables set in your terminal or `.bashrc` file before taking the steps below.*


.. code:: bash

  mkdir <directory_for_workspaces>
  cd <directory_for_workspaces>
  wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/initial_ros_setup.sh
  chmod a+x initial_ros_setup.sh
  ./initial_ros_setup.sh


**Summary of what's being done**

The ``initial_ros_setup.sh`` script downloads four ROS workspaces and then builds them in the correct order. The four workspaces are:

- **ROS 2 release**: This is the latest ROS 2 release as defined by the repos file found `here <https://github.com/ros2/ros2>`_
- **ROS 2 dependencies**: This is a set of ROS 2 packages that aren't included in the ROS 2 release yet. However, you need them to be able to build Navigation2. This also includes packages that are part of the ROS 2 release where Navigation2 uses a different version.
- **Navigation2**: This repository.

After all the workspaces are downloaded, run the `navigation2/tools/build_all.sh` script. `build_all.sh` builds each repo in the order listed above using the `colcon build --symlink-install` command.

Options
=======

The `initial_ros_setup.sh` accepts the following options:

- `--no-ros2` This skips downloading and building the ROS 2 release. Instead it uses the binary packages and ``setup.sh`` installed in ``/opt/ros/<ros2-distro>``
- ``--download-only`` This skips the build steps
