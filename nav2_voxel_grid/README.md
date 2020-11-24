# Nav2 Voxel Grid

The `nav2_voxel_grid` package contains the VoxelGrid used by the `Voxel Layer` inside of `nav2_costmap_2d`. The voxel grid itself is simply a 2D char pointer array of the map size with bit locations corresponding to voxel values (free, unknown, occupied , etc). 

It is branched out as a separate package for use in other applications where a dense voxel grid representation may be useful. It also contains implementations of 3D raycasting. 

## ROS1 Comparison

This package is a direct port to ROS2 for use in the voxel layer. 
