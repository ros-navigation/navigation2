#include "nav2_msgs/msg/costmap.hpp"             
#include "nav2_collision_monitor/costmap.hpp"   
#include <functional>
#include <cmath>
#include <tf2/time.hpp>                     // tf2::Duration, tf2::TimePoint, durationFromSec()
#include <tf2_ros/buffer.hpp>               // tf2_ros::Buffer (you already use it)
#include <tf2_ros/transform_listener.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>  // nav2::LifecycleNode
#include <nav2_ros_common/node_utils.hpp>

namespace nav2_collision_monitor
{


CostmapSource::CostmapSource(                     
  // const nav2_util::LifecycleNode::WeakPtr & node,
  const nav2::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating CostmapSource", source_name_.c_str());  
}


CostmapSource::~CostmapSource()                                                         
{
  RCLCPP_INFO(logger_, "[%s]: Destroying CostmapSource", source_name_.c_str());       
  data_sub_.reset();
}

void CostmapSource::configure()                                                         
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  std::string source_topic;
  getParameters(source_topic);
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

 
  data_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
      source_topic,
     std::bind(&CostmapSource::dataCallback, this, std::placeholders::_1),
      qos);
}

bool CostmapSource::getData(                                                            
  const rclcpp::Time & curr_time,
  std::vector<Point> & data)
{
  if (data_ == nullptr) {
    return false;
  }

  if (!sourceValid(data_->header.stamp, curr_time)) {
    return false;
  }
  tf2::Transform tf_transform;
  if (!getTransform(curr_time, data_->header, tf_transform)) {
    return false;
  }

  // Extract lethal/inscribed cells and transform to base frame
  const auto & cm = *data_;                                                             
  const auto & meta = cm.metadata;                                                      
  
  for (unsigned int y = 0; y < meta.size_y; ++y) {                                      
    for (unsigned int x = 0; x < meta.size_x; ++x) {                                    
      const int idx = y * meta.size_x + x;                                              

      // CHANGE: Costmap costs are 0..255 (0 free, 253 inscribed, 254 lethal, 255 unknown)
      if (cm.data[idx] >= cost_threshold_) {                                            
        const double wx = meta.origin.position.x + (x + 0.5) * meta.resolution;        
        const double wy = meta.origin.position.y + (y + 0.5) * meta.resolution;        
        tf2::Vector3 p_v3_s(wx, wy, 0.0);
        tf2::Vector3 p_v3_b = tf_transform * p_v3_s;
        data.push_back({p_v3_b.x(), p_v3_b.y()});
      }
    }
  }
  return true;
}

void CostmapSource::getParameters(std::string & source_topic)                           
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  getCommonParameters(source_topic);



  // Todo: use a cost threshold suitable for Nav2 costmaps; default 254 (lethal)
  nav2::declare_parameter_if_not_declared(
    node, source_name_ + ".cost_threshold", rclcpp::ParameterValue(254));               
  cost_threshold_ = node->get_parameter(source_name_ + ".cost_threshold").as_int();     
}

void CostmapSource::dataCallback(nav2_msgs::msg::Costmap::ConstSharedPtr msg)          
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor