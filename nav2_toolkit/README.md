# Nav2 Toolkit

A collection of modular utility tools to enhance ROS 2 Navigation2 applications.

## Overview

The Nav2 Toolkit provides quality-of-life utilities that complement the Navigation2 stack with additional functionality. These tools aim to improve automation, reduce manual intervention, and enhance the overall navigation experience in robotics applications.

## Features

### Pose Persistence

Save and restore robot poses across system restarts:

- **Periodic Pose Saving**: Automatically saves the robot's current pose to disk
- **Position Recovery**: Restores the robot's last known position after system restarts
- **Crash-Safe Storage**: Uses atomic file operations to prevent data corruption
- **Service Interface**: Start/stop saving, force immediate save, restore from saved data

### Base Footprint Publisher

## Configuration Parameters

### Pose Persistence

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `save_interval` | double | 5.0 | Time in seconds between pose saves |
| `pose_file` | string | ~/.ros/pose.yaml | File path to store pose data |
| `auto_save_on_startup` | bool | true | Automatically start saving poses on node startup |
| `auto_restore_on_startup` | bool | false | Automatically restore pose on node startup |

