# Nav2 Fake Localization 

fake_localization provides a subset of ros apis to substitutes a localization system used by amcl. This node is most frequently used during simulation as a method to provide perfect localization in a computationally inexpensive manner.

It provides a transform from `global_frame_id` to `odom_frame_id`, and accepts `/initialpose` to reinitialise its simulated pose.
