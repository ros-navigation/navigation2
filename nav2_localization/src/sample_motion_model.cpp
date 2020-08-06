#include "pluginlib/class_list_macros.hpp"
#include "interfaces/sample_motion_model_base.hpp"
#include "plugins/sample_motion_model_plugins.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_localization_plugins::DummyMotionSampler, nav2_localization_base::SampleMotionModel)