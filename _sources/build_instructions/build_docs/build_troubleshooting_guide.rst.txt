.. _build-troubleshooting-guide:

Build Troubleshooting Guide
**********************************************

Common Navigation2 Dependencies Build Failures
==============================================

* Makse sure that .bashrc file has no ROS environment variables in it. Open new terminals and try to build the packages again.

* Make sure to run rosdep for the correct ROS 2 distribution.
  ``rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>``

* Make sure that the ``setup.bash`` is sourced in the ROS 2 installation or ROS 2 build workspace. Check if you can run talker and listener nodes.

* Make sure that the ``setup.bash`` in ``nav2_depend_ws/install`` is sourced.

* Check if you have the correct ROS version and distribution. ``printenv | grep -i ROS``

* Make sure to run rosdep for the correct ROS 2 distribution.

  ``rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>``

* Search `GitHub Issues <https://github.com/ros-planning/navigation2/issues>`_

Still can't solve it? Let us know about your issue. `Open a ticket <https://github.com/ros-planning/navigation2/issues/new>`_.

Reporting Issue
===============

- If you run into any issues when building Navigation2, you can use the search tool in the issues tab on `GitHub <https://github.com/ros-planning/navigation2/issues>`_ and always feel free to `open a ticket <https://github.com/ros-planning/navigation2/issues/new>`_.
