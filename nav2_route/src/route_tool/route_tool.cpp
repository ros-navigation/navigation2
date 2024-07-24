#include "nav2_route/route_tool/route_tool.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QDesktopServices>
#include <QUrl>
#include <unistd.h>
#include <sys/types.h>
#include <QFileDialog>
#include <pwd.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(route_tool::routeTool, rviz_common::Panel)

namespace route_tool
{
    routeTool::routeTool(QWidget * parent)
    :   rviz_common::Panel(parent),
        ui_(std::make_unique<Ui::route_tool>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);
        node_ = std::make_shared<nav2_util::LifecycleNode>("route_tool_node", "", rclcpp::NodeOptions());
        node_->configure();
        graph_vis_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "route_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        node_->activate();
        tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        graph_loader_ = std::make_shared<nav2_route::GraphLoader>(node_, tf_, "map");
    }

    void routeTool::update_display(void)
    {
    }

    void routeTool::on_load_button_clicked(void)
    {
        struct passwd *pw = getpwuid(getuid());
        std::string homedir(pw->pw_dir);
        QString filename = QFileDialog::getOpenFileName(this,
        tr("Open Address Book"), "",
        tr("Address Book (*.geojson);;All Files (*)"));
        load_route_graph(filename.toStdString());
    }

    void routeTool::on_save_button_clicked(void)
    {
        // TODO
    }

    void routeTool::on_create_button_clicked(void)
    {

    }

    void routeTool::on_submit_button_clicked(void)
    {

    }

    void routeTool::on_delete_button_clicked(void)
    {

    }

    void routeTool::load_route_graph(std::string filename)
    {
        graph_loader_->loadGraphFromFile(graph_, graph_to_id_map_, filename);
        graph_vis_publisher_->publish(nav2_route::utils::toMsg(graph_, "map", node_->now()));
    }

    void routeTool::save_route_graph(void)
    {
        // TODO
    }

    void routeTool::load_map(std::string filename)
    {
        RCLCPP_INFO(node_->get_logger(), "Map: %s", filename.c_str());
    }

    void routeTool::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    void routeTool::load(const rviz_common::Config & config)
    {
        rviz_common::Panel::load(config);
    }
} // namespace route_tool
