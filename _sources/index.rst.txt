.. _documentation_home:

*****
|LPN|
*****

Overview
########

The |PN| project is a set of ROS2 components that help you build a robot that
can drive around. The ROS 2 Navigation System is the framework that
enables a robot to autonomously reach a goal state, such as a position
and orientation relative to a map. Given a current pose, a map,
and a goal, the navigation system generates a plan to reach the goal,
and outputs commands to autonomously drive the robot,
respecting any safety constraints and avoiding obstacles.

It has tools to:

- load and store maps
- localize the robot on the map
- plan a path from A to B around obstacles
- control the robot as it follows the path and avoid any obstacles along the way
- convert sensor data into a costmap representation of the world
- build complicated robot behaviors using behavior trees
- Plugins to enable your own custom algorithms and behaviors

These all work seemlessly together, so that, with a bit of customization for your
robot, you can just give it a destination and off it go.

Here you will find documentation on how to install and use |PN| with an example robot, Turtlebot
3, as well as how to customize it for other robots, tune the behavior for better
performance, as well as customize the internals for advanced results.

.. raw:: html

    <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
      <iframe width="708" height="400" src="https://www.youtube.com/embed/ZeCds7Sv-5Q" frameborder="0" allowfullscreen></iframe>
    </div>

.. image:: images/architectural_diagram.png
    :height: 400px
    :width: 708px
    :alt: Navigation2 Block Diagram

Sections
********

.. toctree::
   :maxdepth: 1

   getting_started/index.rst
   howtos/index.rst
   tutorials/index.rst
   release_notes.rst
   contribute/index.rst
