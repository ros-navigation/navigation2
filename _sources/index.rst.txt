.. _documentation_home:

*****
|LPN|
*****

Overview
########

The Navigation 2 project is the spiritual successor of the ROS Navigation Stack.
This project seeks to find a safe way to have a mobile robot move from point A to
point B. This will complete dynamic path planning, compute velocities for motors,
avoid obstacles, and structure recovery behaviors.

Navigation 2 is built on ROS2 and makes use of Behavior Trees for navigation
task orchestration, rather than ``move_base``'s finite state machine.
For those familiar with the ROS1 Navigation Stack will know that ``move_base``
had some issues and we have addressed them in Navigation 2.

Navigation 2 uses behavior trees to call modular servers to complete an action.
An action can be to compute a path, control effort, recovery, or any other navigation
related action. These are each separate nodes that communicate with the behavior tree (BT)
over a ROS action server. The diagram below will give you a good first-look at the structure
of Navigation 2. Note: It is possible to have multiple plugins for controllers, planners,
and recoveries in each of their servers with matching BT plugins. This can be used to
create contextual navigation behaviors.

The expected inputs to Navigation2 (Nav2) are TF transformations conforming to REP-105, a
map source if utilizing the Static Costmap Layer, a BT XML file, and any relevant sensor data
sources. It will then provide valid velocity commands for the motors of a holonomic or
non-holonomic robot to follow. We currently support holonomic and differential-drive base types
but plan to support Ackermann (car-like) robots as well in the near future.


It has tools to:

- load, serve, and store maps (Map Server)
- localize the robot on the map (AMCL)
- plan a path from A to B around obstacles (Nav2 Planner)
- control the robot as it follows the path (Nav2 Controller)
- convert sensor data into a costmap representation of the world (Nav2 Costmap 2D)
- build complicated robot behaviors using behavior trees (Nav2 Behavior Trees and BT Navigator)
- Compute recovery behaviors in case of failure (Nav2 Recoveries)
- Follow sequential waypoints (Nav2 Waypoint Follower)
- Manage the lifecycle of the servers (Nav2 Lifecycle Manager)
- Plugins to enable your own custom algorithms and behaviors (Nav2 Core)

.. image:: images/architectural_diagram.png
    :height: 400px
    :width: 708px
    :alt: Navigation2 Block Diagram

We also provide a set of starting plugins to get you going. NavFn computes the shortest path
from a pose to a goal pose using A* or Dijkstra's algorithm. DWB will use the DWA algorithm
to compute a control effort to follow a path, with several plugins of its own for trajectory
critics. There are recovery behaviors included: waiting, spinning, clearing costmaps, and
backing up. There are a set of BT plugins for calling these servers and computing conditions.
Finally, there are a set of Rviz plugins for interacting with the stack and controlling the
lifecycle.

Here is the documentation on how to install and use |PN| with an example robot, Turtlebot
3 (TB3), as well as how to customize it for other robots, tune the behavior for better
performance, as well as customize the internals for advanced results. Below is an example
of the TB3 navigating in a small lounge.

.. raw:: html

    <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
      <iframe width="708" height="400" src="https://www.youtube.com/embed/ZeCds7Sv-5Q" frameborder="0" allowfullscreen></iframe>
    </div>

Sections
********

.. toctree::
   :maxdepth: 1

   getting_started/index.rst
   build_instructions/index.rst
   tutorials/index.rst
   configuration/index.rst
   contribute/index.rst
