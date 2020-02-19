.. _master-build:

Build Navigation2 Master
========================

.. note::

   If you'd rather use an automated method, all the steps here can be completed through ``initial_ros_setup.sh``.
   The steps to use this are outlined in :ref:`quick-start`.

Build ROS 2 Master
------------------

.. warning::

   When building ROS 2 from source, make sure that the `ros2.repos` file is from the `master` branch.

Build ROS2 master using the `build instructions <https://index.ros.org/doc/ros2/Installation>`_ provided in the ROS2 documentation.


Build Navigation2 Dependencies
------------------------------

Since we're not building for a released distribution, we must build the dependencies ourselves rather than using binaries.
First, source the setup.bash file in the ROS 2 build workspace.

    ``source ~/ros2_ws/install/setup.bash``

Next, we're going to get the ``ros2_dependencies.repos`` file from Navigation2.
Then, use ``vcs`` to clone the repos and versions in it into a workspace.

.. code:: bash

  source ros2_ws/install/setup.bash
  mkdir -p ~/nav2_depend_ws/src
  cd ~/nav2_depend_ws
  wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/ros2_dependencies.repos
  vcs import src < ros2_dependencies.repos
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Build Navigation2 Master
------------------------

Finally, now that we have ROS2 master and the necessary dependencies, we can now build Navigation2 master itself.
We'll source the ``nav2_depend_ws``, which will also source the ROS2 master build workspace packages, to build with dependencies.
The rest of this should look familiar.

.. code:: bash

  source ~/nav2_depend_ws/install/setup.bash
  mkdir -p ~/navigation2_ws/src
  cd ~/navigation2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch master
  cd ~/navigation2_ws
  colcon build --symlink-install
