.. _specific-distro-build:

Build Navigation2
=================

Install ROS
-----------

Please install ROS2 via the usual `build instructions <https://index.ros.org/doc/ros2/Installation>`_ for your desired distribution.

Build Navigation2
-----------------

We're going to create a new workspace, ``navigation2_ws``, clone our Navigation2 branch into it, and build.
``rosdep`` will be used to get the dependency binaries for navigation2 in your specific distribution.

.. code:: bash

  mkdir -p ~/navigation2_ws/src
  cd ~/navigation2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch eloquent-devel
  cd ~/navigation2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
  colcon build --symlink-install

Note: You need to change ``--rosdistro`` to the selected ROS 2 distribution name (e.g ``eloquent``).
