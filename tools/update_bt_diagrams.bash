#!/bin/bash

# Run this from the root of the workspace to update these behavior_tree images
# in the doc directory of the nav2_bt_navigator package
navigation2/tools/bt2img.py \
  --behavior_tree navigation2/nav2_bt_navigator/behavior_trees/navigate_w_replanning.xml \
  --image_out navigation2/nav2_bt_navigator/doc/simple_parallel \
  --legend navigation2/nav2_bt_navigator/doc/legend
navigation2/tools/bt2img.py \
  --behavior_tree navigation2/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml \
  --image_out navigation2/nav2_bt_navigator/doc/parallel_w_recovery
navigation2/tools/bt2img.py \
  --behavior_tree navigation2/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_round_robin_recovery.xml \
  --image_out navigation2/nav2_bt_navigator/doc/parallel_w_round_robin_recovery
