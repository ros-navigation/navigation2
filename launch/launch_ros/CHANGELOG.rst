^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2018-08-20)
------------------
* Fixed a bug where launch would hang on exit by destroying the rclpy node on shutdown (`#124 <https://github.com/ros2/launch/issues/124>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Fixed a race condition in emitting events by using loop.call_soon_threadsafe() (`#119 <https://github.com/ros2/launch/issues/119>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Contributors: William Woodall

0.5.2 (2018-07-17)
------------------

0.5.1 (2018-06-27)
------------------
* Various Windows fixes. (`#87 <https://github.com/ros2/launch/issues/87>`_)
* Contributors: William Woodall

0.5.0 (2018-06-19)
------------------
* Changed to use variable typing in comments to support python 3.5 (`#81 <https://github.com/ros2/launch/issues/81>`_)
* First commit of the ROS specific launch API (`#75 <https://github.com/ros2/launch/issues/75>`_)
  * ROS specific functionality for the new launch API.
* Contributors: William Woodall, dhood
