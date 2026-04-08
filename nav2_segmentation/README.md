# nav2_segmentation

`nav2_segmentation` is a bare-minimum discussion skeleton for connecting a Nav2 behavior tree node to a Python ROS 2 action server.

## What It Contains

- A custom `SegmentImage` ROS 2 action
- A C++ Nav2 BT plugin named `SegmentImage`
- A Python action server executable named `nav2_segmentation`
- A sample behavior tree XML

## Behavior Tree Communication

The behavior tree communicates with the package through the `SegmentImage` ROS 2 action. The BT plugin acts as the action client and the Python node hosts the action server on `segment_image` by default.

## Run

```bash
ros2 run nav2_segmentation nav2_segmentation
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

Yes, SAM3 deployment through ONNX generally means exporting/converting the model to ONNX first, then running it via ONNX Runtime (or another ONNX backend).
