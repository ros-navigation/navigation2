.. _getting_started:

Getting Started
###############

This document will take you through the process of installing the |PN| binaries
and navigating a simulated Turtlebot 3 in the Gazebo simulator. We'll use the
|Distro| version of ROS 2, since it is the latest stable version at the time
this was written.

.. tip::

  See the :ref:`howtos` for other situations

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

1. Do stuff
