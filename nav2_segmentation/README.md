# nav2_segmentation

`nav2_segmentation` is a bare-minimum discussion skeleton for connecting a Nav2 behavior tree node to a C++ ROS 2 action server.

## What It Contains

- A custom `SegmentImage` ROS 2 action
- A C++ Nav2 BT plugin named `SegmentImage`
- A C++ action server executable named `nav2_segmentation_server`
- A sample behavior tree XML

## Behavior Tree Communication

The behavior tree communicates with the package through the `SegmentImage` ROS 2 action. The BT plugin acts as the action client and the Python node hosts the action server on `segment_image` by default.
The behavior tree communicates with the package through the `SegmentImage` ROS 2 action. The BT plugin acts as the action client and the C++ node hosts the action server on `segment_image` by default.

## Run

```bash
ros2 run nav2_segmentation nav2_segmentation_server
```

Optional parameter:

- `default_mask_topic`: result mask topic used by dummy response.

## bt_navigator Plugin Configuration

Add the shared library name to `plugin_lib_names`:

```yaml
plugin_lib_names:
  - nav2_segment_image_action_bt_node
```

## Notes

This is dummy code only. There is no real SAM3 segmentation yet.
