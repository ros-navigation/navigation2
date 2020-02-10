.. _specific-distro-build:

Build Navigation2 for a Specific Distribution
=============================================

Step 1- Build ROS 2
-------------------

* `ROS2 Build Instructions <https://index.ros.org/doc/ros2/Installation>`_
* Make sure that the `ros2.repos` file is from the ROS 2 distribution branch that you want to build. (e.g `eloquent-devel`)
* When installing the ROS 2 dependencies using rosdep, make sure to set `--rosdistro` to the desired ROS 2 distribution (e.g `eloquent`)
* The following build instructions are for ROS2 Eloquent. You can change eloquent to your target ROS2 distrubution.

Step 2- Build Navigation2 Dependencies
--------------------------------------

- Source the setup.bash file in the ROS 2 build workspace.

    ``source ~/ros2_ws/install/setup.bash``

**Build and install Navigation2 dependencies:**

.. code:: bash

  mkdir -p ~/nav2_depend_ws/src
  cd ~/nav2_depend_ws
  wget https://raw.githubusercontent.com/ros-planning/navigation2/eloquent-devel/tools/ros2_dependencies.repos
  vcs import src < ros2_dependencies.repos
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


Step 3- Build Navigation2
-------------------------

- To build Navigation2 stack for a specific ROS 2 distribution such as Crystal or Dashing, set ``--branch`` to the selected branch name. (e.g- ``eloquent-devel``)
- Also you need to change ``--rosdistro`` to the selected ROS 2 distribution name. (e.g ``eloquent``)

.. code:: bash

  source ~/nav2_depend_ws/install/setup.bash
  mkdir -p ~/navigation2_ws/src
  cd ~/navigation2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch eloquent-devel
  cd ~/navigation2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
  colcon build --symlink-install
