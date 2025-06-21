// Copyright (c) 2024 John Chrosniak
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_rviz_plugins/route_tool.hpp"
#include <QDesktopServices>
#include <QUrl>
#include <unistd.h>
#include <sys/types.h>
#include <QFileDialog>
#include "rviz_common/display_context.hpp"


namespace nav2_rviz_plugins
{
RouteTool::RouteTool(QWidget * parent)
:   rviz_common::Panel(parent),
  ui_(std::make_unique<Ui::route_tool>())
{
  // Extend the widget with all attributes and children from UI file
  ui_->setupUi(this);
  node_ = std::make_shared<nav2::LifecycleNode>("route_tool_node", "", rclcpp::NodeOptions());
  node_->configure();
  graph_vis_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "route_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  node_->activate();
  tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  graph_loader_ = std::make_shared<nav2_route::GraphLoader>(node_, tf_, "map");
  graph_saver_ = std::make_shared<nav2_route::GraphSaver>(node_, tf_, "map");
  ui_->add_node_button->setChecked(true);
  ui_->edit_node_button->setChecked(true);
  ui_->remove_node_button->setChecked(true);
  // Needed to prevent memory addresses moving from resizing
  // when adding nodes and edges
  graph_.reserve(1000);
}

void RouteTool::onInitialize(void)
{
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    RCLCPP_ERROR(
      node_->get_logger(), "Unable to get ROS node abstraction");
    return;
  }
  auto node = ros_node_abstraction->get_raw_node();

  clicked_point_subscription_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 1, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      ui_->add_field_1->setText(std::to_string(msg->point.x).c_str());
      ui_->add_field_2->setText(std::to_string(msg->point.y).c_str());
      ui_->edit_field_1->setText(std::to_string(msg->point.x).c_str());
      ui_->edit_field_2->setText(std::to_string(msg->point.y).c_str());
    });
}

void RouteTool::on_load_button_clicked(void)
{
  graph_to_id_map_.clear();
  edge_to_node_map_.clear();
  graph_to_incoming_edges_map_.clear();
  graph_.clear();
  QString filename = QFileDialog::getOpenFileName(
    this,
    tr("Open Address Book"), "",
    tr("Address Book (*.geojson);;All Files (*)"));
  graph_loader_->loadGraphFromFile(graph_, graph_to_id_map_, filename.toStdString());
  unsigned int max_node_id = 0;
  for (const auto & node : graph_) {
    max_node_id = std::max(node.nodeid, max_node_id);
    for (const auto & edge : node.neighbors) {
      max_node_id = std::max(edge.edgeid, max_node_id);
      edge_to_node_map_[edge.edgeid] = node.nodeid;
      if (graph_to_incoming_edges_map_.find(edge.end->nodeid) !=
        graph_to_incoming_edges_map_.end())
      {
        graph_to_incoming_edges_map_[edge.end->nodeid].push_back(edge.edgeid);
      } else {
        graph_to_incoming_edges_map_[edge.end->nodeid] = std::vector<unsigned int> {edge.edgeid};
      }
    }
  }
  next_node_id_ = max_node_id + 1;
  update_route_graph();
}

void RouteTool::on_save_button_clicked(void)
{
  QString filename = QFileDialog::getSaveFileName(
    this,
    tr("Open Address Book"), "",
    tr("Address Book (*.geojson);;All Files (*)"));
  RCLCPP_INFO(node_->get_logger(), "Save graph to: %s", filename.toStdString().c_str());
  graph_saver_->saveGraphToFile(graph_, filename.toStdString());
}

void RouteTool::on_create_button_clicked(void)
{
  if (ui_->add_field_1->toPlainText() == "" || ui_->add_field_2->toPlainText() == "") {return;}
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
    graph_[graph_to_id_map_[start_node]].addEdge(
      edge_cost, &(graph_[graph_to_id_map_[end_node]]),
      next_node_id_);
    if (graph_to_incoming_edges_map_.find(end_node) != graph_to_incoming_edges_map_.end()) {
      graph_to_incoming_edges_map_[end_node].push_back(next_node_id_);
    } else {
      graph_to_incoming_edges_map_[end_node] = std::vector<unsigned int> {next_node_id_};
    }
    edge_to_node_map_[next_node_id_++] = start_node;
    RCLCPP_INFO(node_->get_logger(), "Adding edge from %d to %d", start_node, end_node);
    update_route_graph();
  }
  ui_->add_field_1->setText("");
  ui_->add_field_2->setText("");
}

void RouteTool::on_confirm_button_clicked(void)
{
  if (ui_->edit_id->toPlainText() == "" || ui_->edit_field_1->toPlainText() == "" ||
    ui_->edit_field_2->toPlainText() == "") {return;}
  if (ui_->edit_node_button->isChecked()) {
    auto node_id = ui_->edit_id->toPlainText().toInt();
    auto new_longitude = ui_->edit_field_1->toPlainText().toFloat();
    auto new_latitude = ui_->edit_field_2->toPlainText().toFloat();
    if (graph_to_id_map_.find(node_id) != graph_to_id_map_.end()) {
      graph_[graph_to_id_map_[node_id]].coords.x = new_longitude;
      graph_[graph_to_id_map_[node_id]].coords.y = new_latitude;
      update_route_graph();
    }
  } else if (ui_->edit_edge_button->isChecked()) {
    auto edge_id = (unsigned int) ui_->edit_id->toPlainText().toInt();
    auto new_start = ui_->edit_field_1->toPlainText().toInt();
    auto new_end = ui_->edit_field_2->toPlainText().toInt();
    // Find and remove current edge
    auto current_start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
    for (auto itr = current_start_node->neighbors.begin();
      itr != current_start_node->neighbors.end(); itr++)
    {
      if (itr->edgeid == edge_id) {
        current_start_node->neighbors.erase(itr);
        break;
      }
    }
    // Create new edge with same ID using new start and stop nodes
    nav2_route::EdgeCost edge_cost;
    graph_[graph_to_id_map_[new_start]].addEdge(
      edge_cost, &(graph_[graph_to_id_map_[new_end]]),
      edge_id);
    edge_to_node_map_[edge_id] = new_start;
    if (graph_to_incoming_edges_map_.find(new_end) != graph_to_incoming_edges_map_.end()) {
      graph_to_incoming_edges_map_[new_end].push_back(edge_id);
    } else {
      graph_to_incoming_edges_map_[new_end] = std::vector<unsigned int> {edge_id};
    }
    update_route_graph();
  }
  ui_->edit_id->setText("");
  ui_->edit_field_1->setText("");
  ui_->edit_field_2->setText("");
}

void RouteTool::on_delete_button_clicked(void)
{
  if (ui_->remove_id->toPlainText() == "") {return;}
  if (ui_->remove_node_button->isChecked()) {
    unsigned int node_id = ui_->remove_id->toPlainText().toInt();
    // Remove edges pointing to the removed node
    for (auto edge_id : graph_to_incoming_edges_map_[node_id]) {
      auto start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
      for (auto itr = start_node->neighbors.begin(); itr != start_node->neighbors.end(); itr++) {
        if (itr->edgeid == edge_id) {
          start_node->neighbors.erase(itr);
          edge_to_node_map_.erase(edge_id);
          break;
        }
      }
    }
    if (graph_[graph_to_id_map_[node_id]].nodeid == node_id) {
      // Use max int to mark the node as deleted
      graph_[graph_to_id_map_[node_id]].nodeid = std::numeric_limits<int>::max();
      graph_to_id_map_.erase(node_id);
      graph_to_incoming_edges_map_.erase(node_id);
      RCLCPP_INFO(node_->get_logger(), "Removed node %d", node_id);
    }
    update_route_graph();
  } else if (ui_->remove_edge_button->isChecked()) {
    auto edge_id = (unsigned int) ui_->remove_id->toPlainText().toInt();
    auto start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
    for (auto itr = start_node->neighbors.begin(); itr != start_node->neighbors.end(); itr++) {
      if (itr->edgeid == edge_id) {
        RCLCPP_INFO(node_->get_logger(), "Removed edge %d", edge_id);
        start_node->neighbors.erase(itr);
        edge_to_node_map_.erase(edge_id);
        break;
      }
    }
    update_route_graph();
  }
  ui_->remove_id->setText("");
}

void RouteTool::on_add_node_button_toggled(void)
{
  if (ui_->add_node_button->isChecked()) {
    ui_->add_text->setText("Position:");
    ui_->add_label_1->setText("X:");
    ui_->add_label_2->setText("Y:");
  } else {
    ui_->add_text->setText("Connections:");
    ui_->add_label_1->setText("Start Node ID:");
    ui_->add_label_2->setText("End Node ID:");
  }
}

void RouteTool::on_edit_node_button_toggled(void)
{
  if (ui_->edit_node_button->isChecked()) {
    ui_->edit_text->setText("Position:");
    ui_->edit_label_1->setText("X:");
    ui_->edit_label_2->setText("Y:");
  } else {
    ui_->edit_text->setText("Connections:");
    ui_->edit_label_1->setText("Start Node ID:");
    ui_->edit_label_2->setText("End Node ID:");
  }
}

void RouteTool::update_route_graph(void)
{
  graph_vis_publisher_->publish(nav2_route::utils::toMsg(graph_, "map", node_->now()));
}

void RouteTool::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void RouteTool::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}
}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::RouteTool, rviz_common::Panel)
