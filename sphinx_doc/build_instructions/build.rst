Build Instructions
##################

Overview
********

The instructions to build **Navigation2 Stack** and **Navigation2 Dependencies**.

- Build Options
  
  - `Quickstart Using Initial Setup Script`_
  - `Manually Build From Source`_
  
- `Build Troubleshooting Guide`_

**Recommended Platform**

* `Ubuntu 18.04 <http://releases.ubuntu.com/18.04/ubuntu-18.04.3-desktop-amd64.iso>`_

**Build Options**

There two different recommended methods for building Navigation2 stack from source.

**Easy Build**

* An automated method to build Navigation2 in a simple and easy way.
* To build Navigation2 by using the build script, go to `Quickstart using initial setup script`_

Manually Build From Source
**************************

Step 1- Build ROS 2
===================

* `ROS2 Build Instructions <https://index.ros.org/doc/ros2/Installation>`_
* When building ROS 2 from source, if you want to get the latest version of the ROS 2 packages, make sure that the `ros2.repos` file is from the `master` branch.
* When installing the ROS 2 dependencies using rosdep, make sure to set `--rosdistro` to the lastest ROS 2 distribution (e.g `eloquent`)

Step 2- Build Navigation2 Dependencies
======================================

- If you built ROS 2 from source, source the setup.bash file in the ROS 2 build workspace.
  
    ``source ~/ros2_ws/install/setup.bash``

- If you installed ROS 2 from the binary packages, source the setup.bash file in the ROS 2 installation directory.
    
    ``source /opt/ros/<ros2-distro>/setup.bash``

**Build and install Navigation2 dependencies:**

.. code:: bash

  mkdir -p ~/nav2_depend_ws/src
  cd ~/nav2_depend_ws
  wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/ros2_dependencies.repos
  vcs import src < ros2_dependencies.repos
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
  colcon build --symlink-install


Step 3- Build Navigation2
=========================

**I want to build the latest version:**

- To build the lastest Navigation2 stack, set ``--branch`` to ``master``.
- When installing dependencies using rosdep, make sure to set ``--rosdistro`` to the lastest ROS 2 distribution (e.g ``eloquent``)

**I want to build an older version:**

- To build Navigation2 stack for a specific ROS 2 distribution such as Cristal or Dashing, set ``--branch`` to the selected branch name. (e.g- ``dashing-devel``) 
- Also you need to change ``--rosdistro`` to the selected ROS 2 distribution name. (e.g ``dashing``)

.. code:: bash

  source ~/nav2_depend_ws/install/setup.bash
  mkdir -p ~/navigation2_ws/src
  cd ~/navigation2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch master
  cd ~/navigation2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
  colcon build --symlink-install

Step 4- Turtlebot3 (Optional)
=============================

Turtlebot3 is one of the target robot platforms to run Navigation2. You can skip these build instructions for Turtlebot3 packages if Turtlebot3 is not your target platform.

- Install Turtlebot 3

.. code:: bash

  sudo apt install ros-<ros2-distro>-turtlebot3-*

**Recommended:** To get the lastest Turtlebot3 packages and build them from source, follow the instructions in this `E-Manual <http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup>`_

* Make sure you select the correct ROS 2 distribution when following the build steps.

  ``ex: sudo apt install ros-<ros2-distro>-<package-name>``

**Tutorials**

After successfully building Navigation2 from source, you can test it by following `these tutorials <https://github.com/ros-planning/navigation2/tree/master/doc/tutorials>`_

Quickstart using initial setup script
*************************************

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

Use Case Recommendations
------------------------

Developer
---------

For developers, running the `initial_ros_setup.sh` once makes sense. After that, you'll typically want to maintain each repo manually using git.

Most work will be done in the Navigation2 workspace, so just building that will save time.

To build Navigation2 only,

.. code:: bash

  cd <directory_for_workspaces>/navigation2_ws
  source ../nav2_dependencies_ws/install/setup.sh
  colcon build --symlink-install


In the case that the developer wants to change any dependencies, they can run
`<directory_for_workspaces>/navigation2_ws/src/navigation2/tools/build_all.sh` in a clean environment to get everything rebuilt easily

Debugging
---------

To build Navigation2 with build symbols, use colcon build with the following flags and cmake arguments.

.. code:: bash

  source ../nav2_dependencies_ws/install/setup.sh
  cd <directory_for_workspaces>/navigation2_ws
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug


This espesially is helpful when debugging an issue with a debugger such as GDB and QtCreator.

Build System
------------

An automated build system could make a clean directory and run the ``initial_ros_setup.sh`` script each time, however, that will generate a lot of unnecessary load on the upstream repo servers, and result in very long builds.

Instead, it would be better to do an initial download of all the source and dependencies
``./initial_ros_setup.sh --download-only``

Then the CI tool can monitor the Navigation2 repo, update it as necessary, and rebuild using either the ``<directory_for_workspaces>/navigation2_ws/src/navigation2/tools/build_all.sh`` script or by running

.. code:: bash

  cd <directory_for_workspaces>/navigation2_ws/src/navigation2
  source ../ros2_nav_dependencies_ws/install/setup.sh
  colcon build --symlink-install

Build Troubleshooting Guide
***************************

- Check List

  - .bashrc file has no ROS environment variables in it.
  - Open new terminals and try to build the packages again.


Common Navigation2 Dependencies Build Failures
==============================================

* Make sure that the ``setup.bash`` file is sourced in the ROS 2 installation or ROS 2 build workspace. Check if you can run talker and listener nodes.
  
* Make sure to run rosdep for the correct ROS 2 distribution. 
  ``rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>``

Common Navigation2 Build Failures
=================================

* Make sure that the ``setup.bash`` is sourced in the ROS 2 installation or ROS 2 build workspace. Check if you can run talker and listener nodes.

* Make sure that the ``setup.bash`` in ``nav2_depen_ws/install`` is sourced.

* Check if you have the correct ROS version and distribution. ``printenv | grep -i ROS``

* Make sure to run rosdep for the correct ROS 2 distribution.

  ``rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>``

* Search `GitHub Issues <https://github.com/ros-planning/navigation2/issues>`_

Still can't solve it? Let us know about your issue. `Open a ticket <https://github.com/ros-planning/navigation2/issues/new>`_.

Reporting Issue
===============

- If you run into any issues when building Navigation2, you can use the search tool in the issues tab on `GitHub <https://github.com/ros-planning/navigation2/issues>`_ and always feel free to `open a ticket <https://github.com/ros-planning/navigation2/issues/new>`_.
- Check the `Build Troubleshooting Guide`_

