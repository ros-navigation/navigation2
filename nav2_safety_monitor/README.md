# Nav2 Safety Monitor

`nav2_safety_monitor` is a package containing an implementation of a collision monitor for use of a 2D laser scanner, sonars, and contact/bump sensors, the main 3 "safety" sensors on a robot. More can be easily added by creating an instance of the SafetySensor abstract class. 

It looks at the range values of obstacles in a configurable box. If the laser has returns in this box, the robot will be commanded to stop until the obstacle is gone or a timeout is achieved. For contact sensors, it looks for the boolean if in contact to stop. For sonars/range sensors, it is dependent on the non-zero ranges whether it falls into the safety (slow down) region or the collision region. Parameters for all can be seen in the example below.

This may be an issue if you plan to get very close to obstacles and have a large collision box configuration. For applications where docking/interacting with items is required, a service is exposed so that the monitor can be toggled on or off as needed for specific maneuvers and get that state from an application level. 

```
Fully described parameter for the process this belongs to

process:
  process:
    ros__parameters:
      sensors: [laser, sonar1, sonar2, bump]
      laser:
        type: laser
        topic: scan
      sonar1:
        type: sonar
        topic: sonar_1
      sonar2:
        type: sonar
        topic: sonar_2
      bump:
        type: collision
        topic: bump
      collision_zone: [0.1, 0.1, 0.0, 0.0]
      safety_zone: [0.3, 0.3, 0.0, 0.0]
      safety_timeout: 120.0
      scan_timeout: 1.0
      sonar_timeout: 1.0
      global_frame: base_link
      sonar_safety_range: 0.3
      sonar_collision_range: 0.1
```

Example useage:

```
#include <nav2_safety_monitor/nav2_safety_monitor.hpp>

auto node = ...

nav2_safety_monitor::SafetyMonitor monitor(node);
action_in_progress = true;

while (rclcpp.ok() && action_in_progress) {

  // do some stuff with a planner, controller, actuators, ... 

  if (!monitor.isActive()) {
    monitor.activate();
  }

  monitor.process();
  if (monitor.isInCollisionZone()) {
    // STOP!
  } else if (monitor.isInSafetyZone()) {
    // lower velocity scale to limit speed
  }

  if (success) {
    action_in_progress = false;
  }
}

```