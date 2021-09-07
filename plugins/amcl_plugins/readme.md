### Plugin package to add new motion models for amcl localization.

**To add any new motion model-**
- add the motion model function in amcl_plugin.cpp and register the classes as plugins using PLUGINLIB_EXPORT_CLASS macro as done for the existing models.
- Add the plugin information in plugins.xml.
- Specify the new model name(robot_model_type) in the nav2_params.yaml or the yaml file which is used to specify the amcl parameters.A snippet of the yaml file is shown [here](https://github.com/poornimajd/amcl_refactored/blob/main/amcl_yaml_sample.yaml
)
- Build the amcl_plugins package and test the new motion model with amcl source code.
