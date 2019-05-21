// Copyright (c) 2019 Steve Macenski
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

#include "nav2_safety_monitor/nav2_safety_monitor.hpp"

namespace nav2_safety_monitor
{

SafetyMonitor::SafetyMonitor(rclcpp::Node::SharedPtr & node)
: node_(node), active_(true)
{
  toggle_server_ = node_->create_service<std_srvs::srv::Empty>("toggle_safety_monitor",
    std::bind(&SafetyMonitor::toggleCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  std::string topic, config_type;
  node_->get_parameter_or<std::string>("laser_topic", topic, std::string("scan"));
  safety_sensors_.push_back(new LaserSensor(node_, topic));

  rclcpp::Parameter sensors_param = node_->get_parameter("sensors");
  std::vector<std::string> sensors_configs = sensors_param.as_string_array();

  for (uint i=0; i != sensors_configs.size(); i++) {
    config_type = std::string("");
    topic = std::string("");
    node_->get_parameter<std::string>(sensors_configs[i] + "." + "type", config_type);
    node_->get_parameter<std::string>(sensors_configs[i] + "." + "topic", topic);

    if (topic == std::string("") || config_type == std::string("")) {
      RCLCPP_WARN(node_->get_logger(), "No topic given for config %s.", sensors_configs[i].c_str());
    }

    if (config_type == std::string("laser")) {
      RCLCPP_INFO(node_->get_logger(), "Creating laser sensor of topic: %s.", topic.c_str());
      safety_sensors_.push_back(std::make_unique<LaserSensor>(node_, topic));
    } else if (config_type == std::string("collision")) {
      RCLCPP_INFO(node_->get_logger(), "Creating collision sensor of topic: %s.", topic.c_str());
      safety_sensors_.push_back(std::make_unique<CollisionSensor>(node_, topic));
    } else if (config_type == std::string("sonar")) {
      RCLCPP_INFO(node_->get_logger(), "Creating sonar sensor of topic: %s.", topic.c_str());
      safety_sensors_.push_back(std::make_unique<SonarSensor>(node_, topic));
    } else {
      RCLCPP_WARN(node_->get_logger(), "Config: %s's type: %s is invalid. "
        "Options are: laser, collision, or sonar.", sensors_configs[i].c_str(), config_type.c_str());
    }
  }
}

SafetyMonitor::~SafetyMonitor()
{
}

void
SafetyMonitor::toggleCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
  const std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  active_ = !active_;
  RCLCPP_INFO(node_->get_logger(), "Toggling to %s",
    active_ ? "active" : "inactive");
}

void
SafetyMonitor::process()
{
  std::vector<SafetySensor*>::const_iterator sensor = safety_sensors_.begin();
  for ( ; sensor != safety_sensors_.end(); ++sensor) {
    (*sensor)->process();
  }
}

bool
SafetyMonitor::isInCollisionZone()
{
  if (!active_) {
    return false;
  }

  std::vector<SafetySensor*>::const_iterator sensor = safety_sensors_.begin();

  for ( ; sensor != safety_sensors_.end(); ++sensor) {
    if ((*sensor)->getState() == SafetyState::COLLISION) {
      return true;
    }
  }

  return false;
}

bool
SafetyMonitor::isInSafetyZone()
{
  if (!active_) {
    return false;
  }

  std::vector<SafetySensor*>::const_iterator sensor = safety_sensors_.begin();

  for ( ; sensor != safety_sensors_.end(); ++sensor) {
    if ((*sensor)->getState() == SafetyState::SLOW) {
      return true;
    }
  }

  return false;
}

bool
SafetyMonitor::isActive()
{
  return active_;
}

void
SafetyMonitor::activate()
{
  active_ = true;
}

void
SafetyMonitor::deactivate()
{
  active_ = false;
}

}  // namespace nav2_safety_monitor
