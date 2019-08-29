.. _ros1_comparison:

Comparison to Move Base in ROS 1
################################

AMCL and map\_server were ported to ROS2 with minimal functional
changes, but some refactoring.

-  amcl -> :projectfile:`nav2_amcl <nav2_amcl/README.md>`

-  map\_server -> :projectfile:`nav2\_map\_server </nav2_map_server/README.md>`

.. figure:: ./move_base_compare_1.png
   :alt: Move Base 1

   Move Base 1

In addition, move\_base itself has been split into multiple components:

-  :projectfile:`nav2\_bt\_navigator </nav2_bt_navigator/README.md>` (replaces
   move\_base)
-  :projectfile:`nav2\_navfn\_planner </nav2_navfn_planner/README.md>` (replaces
   global\_planner)
-  :projectfile:`nav2\_dwb\_controller </nav2_dwb_controller/README.md>` (replaces
   local\_planner)

.. figure:: ./move_base_compare_2.png
   :alt: Move Base 2

   Move Base 2

The *nav2\_bt\_navigator* replaces move\_base at the top level, with a
*Task* interface to call the global and local planners with a
configurable tree-based action model.

-  Note: the *Task* interface is a temporary proxy for ROS2 *Actions*
   which are not yet implemented. When *Actions* become available, the
   planners will be called through ROS2 *Actions*.

The reason for the change was to make it so that global and local
planners would be *Action Servers* and could be replaced at launch or
run time with other implementations providing the same *Action*.

The *nav2\_bt\_navigator* itself is also a *Task Server* and can also be
replaced with other implementations. It uses *Behavior Trees* to make it
possible to have more complex state machines and to add in recovery
behaviors as additional *Task Servers*. See *nav2\_bt\_navigator* for
that implementation. (currently WIP in `Pull request
91 <https://github.com/ros-planning/navigation2/pull/91>`__)

The *nav2\_navfn\_planner* is ported from the *navfn* package in ROS,
but adds the *Task Server* interface to enable it to be strongly
decoupled from the nav2\_bt\_navigator.

Similarly, the *nav2\_dwb\_controller* is ported from the `dwb
controller <https://github.com/locusrobotics/robot_navigation/tree/master/dwb_local_planner>`__
package, and also adds a *Task Server* interface to also enable it to be
decoupled from the *nav2\_bt\_navigator*.

All these changes make it possible to replace any of these nodes at
launch/run time with any other algorithm that implements that same
interface.

**See each package README.md for more details**
