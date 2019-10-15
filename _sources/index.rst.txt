.. _documentation_home:

*****
|LPN|
*****

.. raw:: html

    <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/ZeCds7Sv-5Q" frameborder="0" allowfullscreen></iframe>
    </div>

Overview
########

The |PN| project is a set of ROS2 components that help you build a robot that
can drive around.

It has tools to:

- load and store maps
- localize the robot on the map
- plan a path from A to B around obstacles
- control the robot as it follows the path and avoid any obstacles along the way
- convert sensor data into a costmap representation of the world
- build complicated robot behaviors using behavior trees

These all work seemlessly together, so that, with a bit of customization for your
robot, you can just give it a destination and off it go.

Here you will find documentation on how to install and use |PN| with a Turtlebot
3, as well as how to customize it for other robots, tune the behavior for better
performance, as well as customize the internals for advanced results.

Sections
********

.. toctree::
   :maxdepth: 1

   getting_started/index.rst
   how_it_works/index.rst
   howtos/index.rst
   release_notes.rst
   contribute/index.rst
