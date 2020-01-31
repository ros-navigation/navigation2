.. _master-build:

Build Navigation2 Master Branch
*******************************

The instructions to build Navigation2 Master branch from source.

Step 1- Build ROS 2
-------------------

* `ROS2 Build Instructions <https://index.ros.org/doc/ros2/Installation>`_
* When building ROS 2 from source, make sure that the `ros2.repos` file is from the `master` branch.

Step 2- Build Navigation2 Dependencies
--------------------------------------

- Source the setup.bash file in the ROS 2 build workspace.

    ``source ~/ros2_ws/install/setup.bash``

**Build and install Navigation2 dependencies:**

.. code:: bash

  mkdir -p ~/nav2_depend_ws/src
  cd ~/nav2_depend_ws
  wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/ros2_dependencies.repos
  vcs import src < ros2_dependencies.repos
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Step 3- Build Navigation2
-------------------------

.. code:: bash

  source ~/nav2_depend_ws/install/setup.bash
  mkdir -p ~/navigation2_ws/src
  cd ~/navigation2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch master
  cd ~/navigation2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
  colcon build --symlink-install
