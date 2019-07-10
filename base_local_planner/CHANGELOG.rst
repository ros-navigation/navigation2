^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package base_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.2 (2018-07-31)
-------------------
* Merge pull request `#773 <https://github.com/ros-planning/navigation/issues/773>`_ from ros-planning/packaging_fixes
  packaging fixes
* add explicit sensor_msgs, tf2 depends for base_local_planner
* Contributors: Michael Ferguson

1.16.1 (2018-07-28)
-------------------

1.16.0 (2018-07-25)
-------------------
* Remove PCL `#765 <https://github.com/ros-planning/navigation/issues/765>`_
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* Fix trajectory obstacle scoring in dwa_local_planner.
* Make trajectory scoring scales consistent.
* unify parameter names between base_local_planner and dwa_local_planner
  addresses parts of `#90 <https://github.com/ros-planning/navigation/issues/90>`_
* fix param to min_in_place_vel_theta, closes `#487 <https://github.com/ros-planning/navigation/issues/487>`_
* add const to getLocalPlane, fixes `#709 <https://github.com/ros-planning/navigation/issues/709>`_
* Merge pull request `#732 <https://github.com/ros-planning/navigation/issues/732>`_ from marting87/small_typo_fixes
  Small typo fixes in ftrajectory_planner_ros and robot_pose_ekf
* Fixed typos for base_local_planner
* Contributors: Alexander Moriarty, David V. Lu, Martin Ganeff, Michael Ferguson, Pavlo Kolomiiets, Rein Appeldoorn, Vincent Rabaud, moriarty

1.15.2 (2018-03-22)
-------------------
* Merge pull request `#673 <https://github.com/ros-planning/navigation/issues/673>`_ from ros-planning/email_update_lunar
  update maintainer email (lunar)
* CostmapModel: Make lineCost and pointCost public (`#658 <https://github.com/ros-planning/navigation/issues/658>`_)
  Make the methods `lineCost` and `pointCost` of the CostmapModel class
  public so they can be used outside of the class.
  Both methods are not changing the instance, so this should not cause any
  problems.  To emphasise their constness, add the actual `const` keyword.
* Merge pull request `#649 <https://github.com/ros-planning/navigation/issues/649>`_ from aaronhoy/lunar_add_ahoy
  Add myself as a maintainer.
* Contributors: Aaron Hoy, Felix Widmaier, Michael Ferguson

1.15.1 (2017-08-14)
-------------------

1.15.0 (2017-08-07)
-------------------
* set message_generation build and runtime dependency
* convert packages to format2
* cleaner logic, fixes `#156 <https://github.com/ros-planning/navigation/issues/156>`_
* Merge pull request `#596 <https://github.com/ros-planning/navigation/issues/596>`_ from ros-planning/lunar_548
* Add cost function to prevent unnecessary spinning
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* add missing deps on libpcl
* import only PCL common
* pcl proagate -lQt5::Widgets flag so we need to find_package Qt5Widgets (`#578 <https://github.com/ros-planning/navigation/issues/578>`_)
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* remove GCC warnings
* Contributors: Lukas Bulwahn, Martin Günther, Michael Ferguson, Mikael Arguedas, Morgan Quigley, Vincent Rabaud, lengly

1.14.0 (2016-05-20)
-------------------

1.13.1 (2015-10-29)
-------------------
* base_local_planner: some fixes in goal_functions
* Merge pull request `#348 <https://github.com/ros-planning/navigation/issues/348>`_ from mikeferguson/trajectory_planner_fixes
* fix stuck_left/right calculation
* fix calculation of heading diff
* Contributors: Gael Ecorchard, Michael Ferguson

1.13.0 (2015-03-17)
-------------------
* remove previously deprecated param
* Contributors: Michael Ferguson

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------
* Add ARCHIVE_DESTINATION for static builds
* Contributors: Gary Servin

1.11.14 (2014-12-05)
--------------------
* Fixed setting child_frame_id in base_local_planner::OdometryHelperRos
* Contributors: Mani Monajjemi

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* Bugfix uninitialised occ_cost variable usage
  This fixes `#256 <https://github.com/ros-planning/navigation/issues/256>`_.
* base_local_planner: adds waitForTransform
* Fixed issue causing trajectory planner returning false to isGoalReach ed even when it's control thread finishes executing
* Contributors: Daniel Stonier, Marcus Liebhardt, hes3pal

1.11.11 (2014-07-23)
--------------------
* Minor code cleanup
* Contributors: Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
* renames acc_lim_th to acc_lim_theta, add warning if using acc_lim_th
* uses odom child_frame_id to set robot_vel frame_id
* Contributors: David Lu!!, Michael Ferguson, Enrique Fernández Perdomo

1.11.9 (2014-06-10)
-------------------
* uses ::hypot(x, y) instead of sqrt(x*x, y*y)
* No need to use `limits->`
* Contributors: Enrique Fernández Perdomo

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------
* fixes latch_xy_goal_tolerance param not taken
* update build to find eigen using cmake_modules
* Trajectory: fix constness of getter methods
* Use hypot() instead of sqrt(x*x, y*y)
* Fix bug in distance calculation for trajectory rollout
* Some documentation fixes in SimpleTrajectoryGenerator
* Contributors: Michael Ferguson, Siegfried-A. Gevatter Pujals, enriquefernandez

1.11.5 (2014-01-30)
-------------------
* Merge pull request `#152 <https://github.com/ros-planning/navigation/issues/152>`_ from KaijenHsiao/hydro-devel
  uncommented trajectory_planner_ros from catkin_package LIBRARIES so other packages can find it
* Fix negative score bug, add ability to sum scores
* Ignore pyc files from running in devel
* Correct type of prefer_forward penalty member variable
* uncommented trajectory_planner_ros from catkin_package LIBRARIES so other packages can find it
* Better handling of frame param in MapGridVisualizer
* check for CATKIN_ENABLE_TESTING
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* Changed new Odom-Helper::initialize() function to setOdomTopic().
* Converted to a pointcloud pointer in Observation in more places.
