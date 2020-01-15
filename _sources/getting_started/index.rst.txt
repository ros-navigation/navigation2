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

      sudo apt install ros-dashing-navigation2
      sudo apt install ros-dashing-nav2-bringup

3. Install the Turtlebot 3 packages. Again, for Ubuntu 18, it looks like this:

   .. code-block:: bash

      sudo apt install ros-dashing-turtlebot3*

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

      ros2 launch nav2_bringup nav2_simulation_launch.py


   If everything has started correctly, you will see the RViz and Gazebo GUIs like
   this.

   .. figure:: /images/rviz/rviz-not-started.png
      :scale: 50%
      :alt: Initial appearance of RViz before hitting startup button

      Initial appearance of RViz before hitting startup button. Nothing is
      displayed at this point because |PN| is still in the unconfigured state

   .. figure:: /images/gazebo/gazebo_turtlebot1.png
      :scale: 50%
      :alt: Initial appearance of Gazebo with Turtlebot 3 world

      Initial appearance of Gazebo with Turtlebot 3 world

4. Click the "Startup" button in the bottom left corner of RViz. This will
   cause |PN| to change to the Active lifecycle state. It should
   change appearance to show the map.

   .. figure:: /images/rviz/rviz_initial.png
      :scale: 50%
      :alt: Initial appearance of RViz transitioning to the Active state

      Initial appearance of RViz transitioning to the Active state

Navigating
**********

1. After starting, the robot initially has no idea where it is. By default,
   |PN| waits for you to give it an approximate starting position. Take a look
   at where the robot is in the Gazebo world, and find that spot on the map. Set
   the initial pose by clicking the "2D Pose Estimate" button in RViz, and then
   down clicking on the map in that location. You set the orientation by dragging
   forward from the down click.

   If you are using the defaults so far, it should look like this.

   .. figure:: /images/rviz/rviz-set-initial-pose.png
      :scale: 50%
      :alt: Approximate starting location of Turtlebot

      Approximate starting location of Turtlebot

   If you don't get the location exactly right, that's fine. |PN| will refine
   the position as it navigates. You can also, click the "2D Pose
   Estimate" button and try again, if you prefer.

   Once you've set the initial pose, the trasform tree will be complete and
   |PN| is fully active and ready to go.

   .. figure:: /images/rviz/navstack-ready.png
      :scale: 50%
      :alt: |PN| is ready. Transforms and Costmap show in RViz.

      |PN| is ready. Transforms and Costmap show in RViz.

2. Click the "Navigaton2 Goal" button and choose a destination.

   .. figure:: /images/rviz/navigate-to-pose.png
      :scale: 50%
      :alt: Setting the goal pose in RViz.

      Setting the goal pose in RViz.

   Watch the robot go!

   .. figure:: /images/rviz/navigating.png
      :scale: 50%
      :alt: Turtlebot on its way to the goal.

      Turtlebot on its way to the goal.
