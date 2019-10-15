.. _getting_started:

Getting Started
###############

This document will take you through the process of installing the |PN| binaries
and navigating a simulated Turtlebot 3 in the Gazebo simulator. We'll use the
|Distro| version of ROS 2, since it is the latest stable version at the time
this was written. The instructions are written primarily for Ubuntu 18, using
the standard installation options.

.. note::

  See the :ref:`howtos` for other situations such as building from source or
  working with other types of robots.

.. warning::

  This is a simplified version of the Turtlebot 3 instructions. We highly
  recommend you follow the `official Turtlebot 3 manual`_ if you intend to
  continue working with this robot beyond the minimal example provided here.

Installation
************

1. Install the `ROS 2 binary packages`_ as described in the official docs
2. Install the |PN| packages using your operating system's package manager. For
   Ubuntu 18, do this:

   .. code-block:: bash

      sudo apt install ros2-dashing-navigation2
      sudo apt install ros2-dashing-nav2-bringup

3. Install the Turtlebot 3 packages. Again, for Ubuntu 18, it looks like this:

   .. code-block:: bash

      sudo apt install ros2-dashing-turtlebot3*

Running the Example
*******************

1. Start a terminal in your GUI
2. Set key environment variables. Here's how to do it in Ubuntu.

   .. code-block:: bash

      source /opt/ros/dashing/setup.bash
      export TURTLEBOT3_MODEL=waffle
      export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/dashing/share/turtlebot3_gazebo/models

3. In the same terminal, run

   .. code-block:: bash

      ros2 launch nav2_bringup nav2_tb3_simulation_launch.py
