#include "nav2_localization/nav2_localization.hpp"
#include "nav2_util/string_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

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
    declare_parameter("laser_scan_topic_", "scan");
    declare_parameter("odom_frame_id", "odom");
}

LocalizationServer::~LocalizationServer()
{
    RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
LocalizationServer::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring localization interface");
    
    auto node = shared_from_this();

    initParameters();
    initTransforms();
    initMessageFilters();
    initPubSub();
    
    
    
    

    // TODO: Add matcher


    try {
        sample_motion_model_type_ = nav2_util::get_plugin_type_param(node, sample_motion_model_id_);
        sample_motion_model_ = sample_motion_model_loader_.createUniqueInstance(sample_motion_model_type_);
    } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(get_logger(), "Failed to create sample motion model. Exception: %s", ex.what());
        exit(-1);
    }

    // IS THIS ACTUALLY NEEDED?
    odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);


    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
    odom_sub_.reset();
    laser_scan_connection_.disconnect();
    laser_scan_filter_.reset();
    laser_scan_sub_.reset();

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
LocalizationServer::initParameters()
{
    get_parameter("sample_motion_model_id", sample_motion_model_id_);
    get_parameter("first_map_only", first_map_only_);
    get_parameter("laser_scan_topic", scan_topic_);
    get_parameter("odom_frame_id", odom_frame_id_);

    odom_frame_id_ = nav2_util::strip_leading_slash(odom_frame_id_);
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

void
LocalizationServer::initMessageFilters()
{
    laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        rclcpp_node_.get(), scan_topic_, rmw_qos_profile_sensor_data);

    laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10, rclcpp_node_);

    laser_scan_connection_ = laser_scan_filter_->registerCallback(
        std::bind(
            &LocalizationServer::laserReceived,
            this, std::placeholders::_1));
}

void
LocalizationServer::initTransforms()
{
    // Initilize transform listener and broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
}

void
LocalizationServer::initPubSub()
{
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&LocalizationServer::mapReceived, this, std::placeholders::_1));
}

void
LocalizationServer::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{

}

} //nav2_localiztion