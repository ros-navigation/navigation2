#ifndef POSE_SAVER__POSE_SAVER_NODE_HPP_
#define POSE_SAVER__POSE_SAVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

namespace nav2_toolkit
{

class PoseSaverNode : public rclcpp::Node
{
public:
    explicit PoseSaverNode(const rclcpp::NodeOptions & options);

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void timer_callback();
  void start_service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void stop_service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void restore_service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void reset_pose_file_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void write_pose_to_file(const std::string &filepath);
  void amcl_monitor_callback();
  bool restore_pose_from_file_and_publish();

  geometry_msgs::msg::PoseWithCovarianceStamped read_pose_from_file(const std::string &filepath);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr amcl_monitor_timer_;
  // rclcpp::TimerBase::SharedPtr amcl_check_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_;
  rclcpp::Time last_amcl_time_;
  std::string pose_file_path_;

  bool saving_active_;
  bool auto_restore_;
  int last_sub_count_;
};

} 

#endif  
