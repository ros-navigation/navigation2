#include <chrono>
#include <fstream>
#include <memory>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav2_pose_saver/pose_saver_node.hpp"

using namespace std::chrono_literals;

namespace nav2_pose_saver
{

PoseSaverNode::PoseSaverNode()
: Node("pose_saver_node"), saving_active_(false)
{
  this->declare_parameter<double>("save_interval_sec", 5.0);
  this->declare_parameter<std::string>("pose_file_path", std::string(std::getenv("HOME")) + "/.ros/last_known_pose.yaml");
  this->declare_parameter<bool>("auto_start_saving", true);
  this->declare_parameter<bool>("auto_restore_pose", true);

  pose_file_path_ = this->get_parameter("pose_file_path").as_string();
  double interval_sec = this->get_parameter("save_interval_sec").as_double();
  bool auto_start = this->get_parameter("auto_start_saving").as_bool();
  auto_restore_ = this->get_parameter("auto_restore_pose").as_bool();

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(interval_sec),
    std::bind(&PoseSaverNode::timer_callback, this));
  timer_->cancel();  // Disabled until service starts

  sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&PoseSaverNode::pose_callback, this, std::placeholders::_1));

  initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  last_sub_count_ = 0;
  amcl_monitor_timer_ = this->create_wall_timer(
      2s,  // check every 2 seconds
      std::bind(&PoseSaverNode::amcl_monitor_callback, this));

  start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/start_pose_saver",
    std::bind(&PoseSaverNode::start_service_cb, this, std::placeholders::_1, std::placeholders::_2));

  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/stop_pose_saver",
    std::bind(&PoseSaverNode::stop_service_cb, this, std::placeholders::_1, std::placeholders::_2));

  restore_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/localise_at_last_known_position",
    std::bind(&PoseSaverNode::restore_service_cb, this, std::placeholders::_1, std::placeholders::_2));

  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/reset_last_known_pose",
    std::bind(&PoseSaverNode::reset_pose_file_cb, this, std::placeholders::_1, std::placeholders::_2));

  if (auto_start) {
    saving_active_ = true;
    timer_->reset();
    RCLCPP_INFO(this->get_logger(), "Pose saving auto-started on launch.");
  }

  if (auto_restore_) {
    RCLCPP_INFO(this->get_logger(), "Auto-restore enabled. Waiting for AMCL...");
  
    // Wait for sim time to become active
    if (this->get_parameter_or("use_sim_time", false)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /clock to start (sim time)...");
      while (rclcpp::ok() && this->now().nanoseconds() == 0) {
        rclcpp::sleep_for(100ms);
      }
    }
  
    // Wait for AMCL to subscribe to /initialpose
    rclcpp::Time start = this->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(15.0);
    while (initial_pose_pub_->get_subscription_count() == 0 &&
           (this->now() - start) < timeout)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for AMCL to subscribe to /initialpose...");
      rclcpp::sleep_for(200ms);
    }
  
    if (initial_pose_pub_->get_subscription_count() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Timeout: AMCL did not subscribe to /initialpose.");
    } else {
      try {
        auto pose_msg = read_pose_from_file(pose_file_path_);
        rclcpp::sleep_for(500ms);
        initial_pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Auto-restored initial pose from file.");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed auto-restore: %s", e.what());
      }
    }
  }
  

  RCLCPP_INFO(this->get_logger(), "Pose Saver Node Initialized.");
}

void PoseSaverNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_pose_ = msg;
}

void PoseSaverNode::timer_callback()
{
  if (!saving_active_ || !last_pose_) return;

  try {
    write_pose_to_file("/tmp/pose_saver.yaml");
    write_pose_to_file(get_package_config_path());
    RCLCPP_DEBUG(this->get_logger(), "Saved pose to /tmp and config path.");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save pose: %s", e.what());
  }
}

void PoseSaverNode::start_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  saving_active_ = true;
  timer_->reset();
  res->success = true;
  res->message = "Pose saving started.";
}

void PoseSaverNode::stop_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  saving_active_ = false;
  timer_->cancel();
  res->success = true;
  res->message = "Pose saving stopped.";
}

void PoseSaverNode::restore_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  try {
    auto pose_msg = read_pose_from_file(pose_file_path_);
    rclcpp::sleep_for(500ms);
    initial_pose_pub_->publish(pose_msg);
    res->success = true;
    res->message = "Pose restored from file.";
    RCLCPP_INFO(this->get_logger(), "Pose restored from file.");
  } catch (const std::exception &e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_WARN(this->get_logger(), "Failed to restore pose: %s", e.what());
  }
}

void PoseSaverNode::reset_pose_file_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "pose" << YAML::BeginMap;
    out << YAML::Key << "position" << YAML::BeginMap
        << YAML::Key << "x" << YAML::Value << 0.0
        << YAML::Key << "y" << YAML::Value << 0.0
        << YAML::Key << "z" << YAML::Value << 0.0
        << YAML::EndMap;
    out << YAML::Key << "orientation" << YAML::BeginMap
        << YAML::Key << "x" << YAML::Value << 0.0
        << YAML::Key << "y" << YAML::Value << 0.0
        << YAML::Key << "z" << YAML::Value << 0.0
        << YAML::Key << "w" << YAML::Value << 1.0
        << YAML::EndMap;
    out << YAML::EndMap;
    out << YAML::EndMap;

    std::vector<std::string> paths = {
      "/tmp/pose_saver.yaml",
      get_package_config_path(),
      pose_file_path_
    };

    for (const auto& path : paths) {
      std::string tmp_path = path + ".tmp";
      std::ofstream tmp_out(tmp_path);
      tmp_out << out.c_str();
      tmp_out.close();
      std::rename(tmp_path.c_str(), path.c_str());
    }

    res->success = true;
    res->message = "Pose reset to zeros.";
    RCLCPP_INFO(this->get_logger(), "Pose reset to default in all paths.");
  } catch (const std::exception &e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Failed to reset pose: %s", e.what());
  }
}

void PoseSaverNode::write_pose_to_file(const std::string &filepath)
{
  if (!last_pose_) return;

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "pose" << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::BeginMap
      << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.position.x
      << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.position.y
      << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.position.z
      << YAML::EndMap;
  out << YAML::Key << "orientation" << YAML::BeginMap
      << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.orientation.x
      << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.orientation.y
      << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.orientation.z
      << YAML::Key << "w" << YAML::Value << last_pose_->pose.pose.orientation.w
      << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;

  std::string tmp_path = filepath + ".tmp";
  std::ofstream tmp_out(tmp_path);
  tmp_out << out.c_str();
  tmp_out.close();

  std::rename(tmp_path.c_str(), filepath.c_str());
}
void PoseSaverNode::amcl_monitor_callback()
{
  int current_count = initial_pose_pub_->get_subscription_count();
  if (last_sub_count_ > 0 && current_count == 0) {
    RCLCPP_WARN(this->get_logger(), "Lost AMCL. Waiting to recover...");
  }
  if (last_sub_count_ == 0 && current_count > 0) {
    RCLCPP_INFO(this->get_logger(), "AMCL reconnected! Publishing last known pose.");
    if (auto_restore_) {
      try {
        auto pose_msg = read_pose_from_file(pose_file_path_);
        rclcpp::sleep_for(500ms);
        initial_pose_pub_->publish(pose_msg);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed to auto-restore after AMCL reconnect: %s", e.what());
      }
    }
  }
  last_sub_count_ = current_count;
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseSaverNode::read_pose_from_file(const std::string &filepath)
{
  YAML::Node node = YAML::LoadFile(filepath);
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = node["pose"]["position"]["x"].as<double>();
  msg.pose.pose.position.y = node["pose"]["position"]["y"].as<double>();
  msg.pose.pose.position.z = node["pose"]["position"]["z"].as<double>();
  msg.pose.pose.orientation.x = node["pose"]["orientation"]["x"].as<double>();
  msg.pose.pose.orientation.y = node["pose"]["orientation"]["y"].as<double>();
  msg.pose.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
  msg.pose.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();
  return msg;
}

std::string PoseSaverNode::get_package_config_path()
{
  return ament_index_cpp::get_package_share_directory("nav2_pose_saver") + "/config/last_known_pose.yaml";
}

}  // namespace nav2_pose_saver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_pose_saver::PoseSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
