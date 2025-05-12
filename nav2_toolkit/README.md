# nav2_toolkit

A modular ROS 2 node for persisting and restoring the robot's last known pose using the `/amcl_pose` topic. This is useful in applications where the robot is never manually moved (e.g. warehouse robots), and it enables automatic re-localization.

## Features
- Save the current AMCL pose to disk periodically
- Write safely using an atomic temp-file swap
- Restore the robot pose automatically on command
- Start/Stop pose saving dynamically with services
- Reset pose file to zeros (e.g. to clear old pose data)
- Parameter configurable save interval and file path
- Support for saving to multiple locations

## Installation

```bash
cd ~/pose_ws/src
git clone https://github.com/<your-username>/nav2_toolkit.git
cd ~/pose_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select nav2_toolkit --symlink-install
source install/setup.bash
```

## Usage

```bash
ros2 launch nav2_toolkit pose_saver.launch.py
```

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `save_interval_sec` | double | `5.0` | Time interval in seconds between pose saves |
| `pose_file_path` | string | `~/.ros/last_known_pose.yaml` | File path used for pose restore/reset services |

## Services

| Service Name | Description |
|--------------|-------------|
| `/start_pose_saver` | Starts periodic pose saving |
| `/stop_pose_saver` | Stops periodic pose saving |
| `/localise_at_last_known_position` | Publishes last saved pose to `/initialpose` |
| `/reset_last_known_pose` | Sets all pose values in the YAML file to zero |

Call a service like this:
```bash
ros2 service call /start_pose_saver std_srvs/srv/Trigger
```

## File Save Locations

When active, poses are saved to:
- `/tmp/pose_saver.yaml`
- `<install_space>/share/nav2_toolkit/config/last_known_pose.yaml`

On restore or reset, the node only uses the path from the `pose_file_path` parameter (default: `~/.ros/last_known_pose.yaml`).

All writes are performed atomically using a `.tmp` file + rename to prevent corruption.

## License

This project is licensed under the [Apache License 2.0](LICENSE).

## Contributing

Contributions are welcome. If you'd like to improve this package, add new features, or fix bugs:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/my-feature`)
5. Open a Pull Request

Please format code using ROS 2 C++ guidelines and keep commits clean.

## Compatibility

- Tested on ROS 2 Jazzy with Turtlebot3
