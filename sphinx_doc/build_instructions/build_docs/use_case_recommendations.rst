.. _use-case-recommendations:

Use Case Recommendations
************************

Developer
---------

For developers, running the `initial_ros_setup.sh` once makes sense. After that, you'll typically want to maintain each repo manually using git.

Most work will be done in the Navigation2 workspace, so just building that will save time.

To build Navigation2 only,

.. code:: bash

  cd <directory_for_workspaces>/navigation2_ws
  source ../nav2_dependencies_ws/install/setup.sh
  colcon build --symlink-install


In the case that the developer wants to change any dependencies, they can run
`<directory_for_workspaces>/navigation2_ws/src/navigation2/tools/build_all.sh` in a clean environment to get everything rebuilt easily

Debugging
---------

To build Navigation2 with build symbols, use colcon build with the following flags and cmake arguments.

.. code:: bash

  source ../nav2_dependencies_ws/install/setup.sh
  cd <directory_for_workspaces>/navigation2_ws
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug


This especially is helpful when debugging an issue with a debugger such as GDB and QtCreator.

Build System
------------

An automated build system could make a clean directory and run the ``initial_ros_setup.sh`` script each time, however, that will generate a lot of unnecessary load on the upstream repo servers, and result in very long builds.

Instead, it would be better to do an initial download of all the source and dependencies
``./initial_ros_setup.sh --download-only``

Then the CI tool can monitor the Navigation2 repo, update it as necessary, and rebuild using either the ``<directory_for_workspaces>/navigation2_ws/src/navigation2/tools/build_all.sh`` script or by running

.. code:: bash

  cd <directory_for_workspaces>/navigation2_ws/src/navigation2
  source ../ros2_nav_dependencies_ws/install/setup.sh
  colcon build --symlink-install

