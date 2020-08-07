#include "nav2_localization/nav2_localization.hpp"

namespace nav2_localization
{

LocalizationServer::LocalizationServer()
: LifecycleNode("localization_server", "", true),
  sample_motion_model_loader_("nav2_localization", "nav2_localization_base::SampleMotionModel"),
  default_sample_motion_model_id_("DummyMotionSampler")
{
    RCLCPP_INFO(get_logger(), "Creating localization server");

    declare_parameter("sample_motion_model_id", default_sample_motion_model_id_);
    declare_parameter("first_map_only", true);
}

LocalizationServer::~LocalizationServer()
{
    RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
LocalizationServer::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring localization interface");

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&LocalizationServer::mapReceived, this, std::placeholders::_1));

    get_parameter("sample_motion_model_id", sample_motion_model_id_);
    get_parameter("first_map_only", first_map_only_);

    // TODO: Add matcher

    auto node = shared_from_this();

    try {
        sample_motion_model_type_ = nav2_util::get_plugin_type_param(node, sample_motion_model_id_);
        sample_motion_model_ = sample_motion_model_loader_.createUniqueInstance(sample_motion_model_type_);
    } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(get_logger(), "Failed to create sample motion model. Exception: %s", ex.what());
        exit(-1);
    }

    odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
    odom_sub_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
LocalizationServer::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_DEBUG(get_logger(), "A new map was received.");
    if (first_map_only_ && first_map_received_) {
        return;
    }
    map_ = *msg;
    first_map_received_ = true;
}

} //nav2_localiztion