# Nav2 Toolkit

**nav2_toolkit** is a collection of modular, production-quality utility nodes for Navigation2 systems. It provides reusable, robust helpers for common navigation tasks.

## Components

### Pose Persistence (PoseSaverNode)

Automatically saves and restores robot pose across system restarts.

**Key Features:**
- Automatic pose backup - Continuously saves pose to disk at configurable intervals
- Intelligent restoration - Restores last known position on startup  
- Crash-safe storage - Atomic file operations prevent corruption during power loss
- Service-based control - Start/stop saving, manual restore, force immediate save

**Use Cases:**
- Multi-session mapping where consistent starting pose is required

### Base Footprint Publisher

Projects 3D robot pose to a 2D navigation frame by removing Z translation, roll, and pitch components. Critical for robots with 3D state estimation operating in 2D navigation environments.

**Key Features:**
- 3D to 2D projection - Strips vertical components for planar navigation
- Real-time transformation - Publishes continuous 2D pose updates
- Configurable frames - Customizable source and target frame names
- Zero latency - Direct transformation without unnecessary buffering

**Use Cases:**
- Aerial robots operating at fixed altitude
- Ground robots with 3D IMU/odometry but 2D navigation requirements
- Mixed 3D/2D sensor fusion systems


## Usage

### Launching with Navigation2

The pose saver integrates directly with Nav2's standard launch system:

```bash
# Launch with Nav2 system (recommended)
ros2 launch nav2_bringup navigation_launch.py

# Override parameters at launch time
ros2 launch nav2_bringup nav2_toolkit_launch.py pose_file_path:=/tmp/pose.yaml save_interval_sec:=1.0

# Standalone launch
ros2 launch nav2_toolkit nav2_toolkit_launch.py
```

### Running Base Footprint Publisher

```bash
# Standard execution
ros2 run nav2_toolkit base_footprint_publisher

# With custom frame names
ros2 run nav2_toolkit base_footprint_publisher --ros-args \
  -p base_link_frame:=robot_base \
  -p base_footprint_frame:=robot_footprint
```

---

## Configuration

### PoseSaverNode Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `save_interval_sec` | `double` | `0.5` | Time between automatic pose saves (seconds) |
| `pose_file_path` | `string` | `~/last_known_pose.yaml` | Full path where pose data is stored |
| `auto_start_saving` | `bool` | `true` | Begin saving poses immediately on startup |
| `auto_restore_pose` | `bool` | `true` | Attempt pose restoration during initialization |

**Configuration Example:**
```yaml
pose_saver:
  ros__parameters:
    save_interval_sec: 1.0
    pose_file_path: "/opt/robot_data/last_pose.yaml"
    auto_start_saving: true
    auto_restore_pose: false
```

### BaseFootprintPublisher Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `base_link_frame` | `string` | `base_link` | Source 3D frame for projection |
| `base_footprint_frame` | `string` | `base_footprint` | Target 2D frame name |

---

## Service Interface

### PoseSaverNode Services

| Service | Type | Description |
|---------|------|-------------|
| `/start_pose_saver` | `std_srvs/srv/Trigger` | Begin automatic pose saving |
| `/stop_pose_saver` | `std_srvs/srv/Trigger` | Stop automatic pose saving |
| `/localise_at_last_known_position` | `std_srvs/srv/Trigger` | Restore saved pose to localization |

**Service Usage Examples:**

```bash
# Stop automatic saving
ros2 service call /stop_pose_saver std_srvs/srv/Trigger

# Restore saved pose
ros2 service call /localise_at_last_known_position std_srvs/srv/Trigger

# Resume automatic saving
ros2 service call /start_pose_saver std_srvs/srv/Trigger
```

---

## Integration

### With Nav2 Launch Files

The toolkit integrates seamlessly with existing Nav2 launch files. Add to your navigation launch:

```xml
<include file="$(find-pkg-share nav2_toolkit)/launch/nav2_toolkit_launch.py">
  <arg name="params_file" value="$(var params_file)"/>
  <arg name="namespace" value="$(var namespace)"/>
</include>
```

### Custom Launch Configuration

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_toolkit',
            executable='pose_saver_node',
            name='pose_saver',
            parameters=[{
                'save_interval_sec': 2.0,
                'pose_file_path': '/persistent/robot_pose.yaml',
                'auto_restore_pose': True
            }]
        ),
        Node(
            package='nav2_toolkit',
            executable='base_footprint_publisher',
            name='footprint_publisher'
        )
    ])
```

---

## File Format

The pose saver creates human-readable YAML files with complete pose information:

```yaml
header:
  frame_id: map
  stamp:
    sec: 179
    nanosec: 600000000
pose:
  position:
    x: 0.31063436209235157
    y: 0.50632403768006418
    z: 0
  orientation:
    x: 0
    y: 0
    z: 0.095350766481568053
    w: 0.99544373589438873
```

---

## Troubleshooting

### Common Issues

**Pose not restoring on startup:**
- Verify `/initialpose` service is available: `ros2 service list | grep initialpose`
- Check pose file exists and is readable: `ls -la ~/last_known_pose.yaml`
- Ensure `auto_restore_pose` parameter is `true`

**File permission errors:**
```bash
# Fix file permissions
sudo chown $USER:$USER ~/last_known_pose.yaml
chmod 644 ~/last_known_pose.yaml
```

**Base footprint transform issues:**
- Verify source frame exists: `ros2 run tf2_ros tf2_echo base_link map`
- Check transform tree: `ros2 run tf2_tools view_frames`

### Debug Logging

Enable detailed logging for troubleshooting:

```bash
ros2 run nav2_toolkit pose_saver_node --ros-args --log-level debug
```

### Service Debugging

Test services manually:

```bash
# Check service availability
ros2 service list | grep pose_saver

# Test service response
ros2 service call /start_pose_saver std_srvs/srv/Trigger "{}"
```


