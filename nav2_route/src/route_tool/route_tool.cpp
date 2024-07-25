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
        graph_loader_->loadGraphFromFile(graph_, graph_to_id_map_, filename.toStdString());
    }

    void routeTool::on_save_button_clicked(void)
    {
        // TODO
    }

    void routeTool::on_create_button_clicked(void)
    {
        if (ui_->add_node_button->isChecked()) {
            auto longitude = ui_->add_field_1->toPlainText().toFloat();
            auto latitude = ui_->add_field_2->toPlainText().toFloat();
            nav2_route::Node new_node;
            new_node.nodeid = next_node_id_;
            new_node.coords.x = longitude;
            new_node.coords.y = latitude;
            graph_.push_back(new_node);
            graph_to_id_map_[next_node_id_++] = graph_.size() - 1;
            RCLCPP_INFO(node_->get_logger(), "Adding node at: (%f, %f)", longitude, latitude);
            update_route_graph();
        } else if (ui_->add_edge_button->isChecked()) {
            auto start_node = ui_->add_field_1->toPlainText().toInt();
            auto end_node = ui_->add_field_2->toPlainText().toInt();
            nav2_route::EdgeCost edge_cost;
            graph_[graph_to_id_map_[start_node]].addEdge(edge_cost, &(graph_[graph_to_id_map_[end_node]]), next_node_id_);
            if (graph_to_incoming_edges_map_.find(end_node) != graph_to_incoming_edges_map_.end()) {
                graph_to_incoming_edges_map_[end_node].push_back(next_node_id_);
            } else {
                graph_to_incoming_edges_map_[end_node] = std::vector<unsigned int> {next_node_id_};
            }
            edge_to_node_map_[next_node_id_++] = start_node;
            RCLCPP_INFO(node_->get_logger(), "Adding edge from %d to %d", start_node, end_node);
            update_route_graph();
        }
    }

    void routeTool::on_confirm_button_clicked(void)
    {
        if (ui_->edit_node_button->isChecked()) {
            auto node_id = ui_->edit_id->toPlainText().toInt();
            auto new_longitude = ui_->edit_field_1->toPlainText().toFloat();
            auto new_latitude = ui_->edit_field_2->toPlainText().toFloat();
            graph_[graph_to_id_map_[node_id]].coords.x = new_longitude;
            graph_[graph_to_id_map_[node_id]].coords.y = new_latitude;
            update_route_graph();
        } else if (ui_->edit_edge_button->isChecked()) {
            auto edge_id = (unsigned int) ui_->edit_id->toPlainText().toInt();
            auto new_start = ui_->edit_field_1->toPlainText().toInt();
            auto new_end = ui_->edit_field_2->toPlainText().toInt();
            // Find and remove current edge
            auto current_start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
            for (auto itr = current_start_node->neighbors.begin(); itr != current_start_node->neighbors.end(); itr++) {
                if (itr->edgeid == edge_id) {
                    RCLCPP_INFO(node_->get_logger(), "Found and removed edge connecting %d and %d", itr->start->nodeid, itr->end->nodeid);
                    current_start_node->neighbors.erase(itr);
                    break;
                }
            }
            // Create new edge with same ID using new start and stop nodes
            nav2_route::EdgeCost edge_cost;
            graph_[graph_to_id_map_[new_start]].addEdge(edge_cost, &(graph_[graph_to_id_map_[new_end]]), edge_id);
            RCLCPP_INFO(node_->get_logger(), "Adding edge from %d to %d", new_start, new_end);
            edge_to_node_map_[edge_id] = new_start;
            if (graph_to_incoming_edges_map_.find(new_end) != graph_to_incoming_edges_map_.end()) {
                graph_to_incoming_edges_map_[new_end].push_back(edge_id);
            } else {
                graph_to_incoming_edges_map_[new_end] = std::vector<unsigned int> {edge_id};
            }
            update_route_graph();
        }

    }

    void routeTool::on_delete_button_clicked(void)
    {
        if (ui_->remove_node_button->isChecked()) {
            auto node_id = ui_->remove_id->toPlainText().toInt();
            // Remove edges pointing to the removed node
            for (auto& edge_id : graph_to_incoming_edges_map_[node_id]) {
                auto start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
                for (auto itr = start_node->neighbors.begin(); itr != start_node->neighbors.end(); itr++) {
                    if (itr->edgeid == edge_id) {
                        RCLCPP_INFO(node_->get_logger(), "Found and removed edge %d", edge_id);
                        start_node->neighbors.erase(itr);
                        edge_to_node_map_.erase(edge_id);
                        break;
                    }
                }
            }
            graph_.erase(graph_.begin() + graph_to_id_map_[node_id]);
            graph_to_id_map_.erase(node_id);
            graph_to_incoming_edges_map_.erase(node_id);
            RCLCPP_INFO(node_->get_logger(), "Removed node %d", node_id);
            update_route_graph();
        } else if (ui_->remove_edge_button->isChecked()) {
            auto edge_id = (unsigned int) ui_->remove_id->toPlainText().toInt();
            auto start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
            for (auto itr = start_node->neighbors.begin(); itr != start_node->neighbors.end(); itr++) {
                if (itr->edgeid == edge_id) {
                    RCLCPP_INFO(node_->get_logger(), "Found and removed edge %d", edge_id);
                    start_node->neighbors.erase(itr);
                    edge_to_node_map_.erase(edge_id);
                    break;
                }
            }
            update_route_graph();
        }
    }

    void routeTool::update_route_graph(void)
    {
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
