.. _build-instructions:

Build and Install
#################

Install
*******

Navigation2 and its dependencies are released as binaries.
You may install it via the following to get the latest stable released version:

  ``sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup ros-<distro>-turtlebot3*``

Build
*****

There are 3 ways to build Navigation2.
Building for a specific released distribution (e.g. ``eloquent``, ``foxy``), build Navigation2 on master branch using a quickstart setup script, or building master branch manually.

.. rst-class:: content-collapse

Build Navigation2 For Released Distribution
===========================================

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
  git clone https://github.com/ros-planning/navigation2.git --branch <ros2-distro>-devel
  cd ~/navigation2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
  colcon build --symlink-install

Note: You need to change ``--rosdistro`` to the selected ROS 2 distribution name (e.g ``eloquent``, ``foxy``).



.. rst-class:: content-collapse

Quickstart Build Master
=======================

Steps
-----

Install all ROS 2 dependencies from the `ROS2 Installation page <https://index.ros.org/doc/ros2/Installation/>`_.
Ensure there are no ROS environment variables set in your terminal or `.bashrc` file before taking the steps below.*


.. code:: bash

  mkdir <directory_for_workspaces>
  cd <directory_for_workspaces>
  wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/initial_ros_setup.sh
  chmod a+x initial_ros_setup.sh
  ./initial_ros_setup.sh


**Summary of what's being done**

The ``initial_ros_setup.sh`` script downloads three ROS workspaces and then builds them in the correct order. The three workspaces are:

- **ROS 2 release**: This is the latest ROS 2 release as defined by the repos file found `here <https://github.com/ros2/ros2>`_
- **ROS 2 dependencies**: This is a set of ROS 2 packages that aren't included in the ROS 2 release yet. However, you need them to be able to build Navigation2. This also includes packages that are part of the ROS 2 release where Navigation2 uses a different version.
- **Navigation2**: This repository.

After all the workspaces are downloaded, run the `navigation2/tools/build_all.sh` script. `build_all.sh` builds each repo in the order listed above using the `colcon build --symlink-install` command.

Options
-------

The `initial_ros_setup.sh` accepts the following options:

- `--no-ros2` This skips downloading and building the ROS 2 release. Instead it uses the binary packages and ``setup.sh`` installed in ``/opt/ros/<ros2-distro>``
- ``--download-only`` This skips the build steps


.. rst-class:: content-collapse

Manually Build Master
=====================

Build ROS 2 Master
------------------

.. warning::

   When building ROS 2 from source, make sure that the `ros2.repos` file is from the `master` branch.

Build ROS2 master using the `build instructions <https://index.ros.org/doc/ros2/Installation>`_ provided in the ROS2 documentation.


Build Navigation2 Dependencies
------------------------------

Since we're not building for a released distribution, we must build the dependencies ourselves rather than using binaries.
First, source the setup.bash file in the ROS 2 build workspace.

    ``source ~/ros2_ws/install/setup.bash``

Next, we're going to get the ``ros2_dependencies.repos`` file from Navigation2.
Then, use ``vcs`` to clone the repos and versions in it into a workspace.

.. code:: bash

  source ros2_ws/install/setup.bash
  mkdir -p ~/nav2_depend_ws/src
  cd ~/nav2_depend_ws
  wget https://raw.githubusercontent.com/ros-planning/navigation2/master/tools/ros2_dependencies.repos
  vcs import src < ros2_dependencies.repos
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Build Navigation2 Master
------------------------

Finally, now that we have ROS2 master and the necessary dependencies, we can now build Navigation2 master itself.
We'll source the ``nav2_depend_ws``, which will also source the ROS2 master build workspace packages, to build with dependencies.
The rest of this should look familiar.

.. code:: bash

  source ~/nav2_depend_ws/install/setup.bash
  mkdir -p ~/navigation2_ws/src
  cd ~/navigation2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch master
  cd ~/navigation2_ws
  colcon build --symlink-install

Docker
******


There are 2 options for docker with Navigation2:
building a container and using the DockerHub container.

.. rst-class:: content-collapse

Building Docker Container
=========================

To build an image from the Dockerfile in the navigation2 folder:
First, clone the repo to your local system (or see Building the source above)

.. code:: bash

  sudo docker build -t nav2/latest .

If proxies are needed:

.. code:: bash

  sudo docker build -t nav2/latest --build-arg http_proxy=http://proxy.my.com:### --build-arg https_proxy=http://proxy.my.com:### .

Note: You may also need to configure your docker for DNS to work. See article here for details: https://development.robinwinslow.uk/2016/06/23/fix-docker-networking-dns/

.. rst-class:: content-collapse

Using DockerHub Container
=========================

We allow for you to pull the latest docker image from the master branch at any time. As new releases and tags are made, docker containers on docker hub will be versioned as well to chose from.

.. code:: bash

  sudo docker pull rosplanning/navigation2:latest

!!!!

Generate Doxygen
****************

Run ``doxygen`` in the root of the Navigation2 repository.
It will generate a ``/doc/*`` directory containing the documentation.
The documentation entrypoint in a browser is index.html.

!!!!

Help
****

:ref:`build-troubleshooting-guide`


.. toctree::
   :hidden:

   build_docs/build_troubleshooting_guide.rst
