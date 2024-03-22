//  Check recieved message is safe or not for the nav2-system
//  For each msg-type known in nav2, we could check it as following:
//  if(!validateMsg()) RCLCPP_ERROR(,"malformed msg. Rejecting.")
//
//  Workflow of validateMsg():
//     if here's a sub-msg-type in the recieved msg,
//        the content of sub-msg would be checked as sub-msg-type
//     then, check the whole recieved msg.
//
//  Following conditions are involved in check:
//     1> Value Check: to avoid damaged value like like `nan`, `INF`
//     2> Logic Check: to avoid value with bad logic, like the size of `map` should be equal to `height*width`
//     3> Any other needed condition could be joint here in future

#ifndef validate_message_HPP
#define validate_message_HPP


#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace nav2_util
{

const int NSEC_PER_SEC = 1e9;  // 1 second = 1e9 nanosecond

bool validateMsg(const builtin_interfaces::msg::Time_<std::allocator<void>> & msg)
{
  // check item
  if (msg.sec < 0) {return false;}                                      // invalid second-stamp
  if (msg.nanosec < 0 || msg.nanosec >= NSEC_PER_SEC) {return false;}   // invalid nanosec-stamp
  return true;
}

bool validateMsg(const std_msgs::msg::Header_<std::allocator<void>> & msg)
{
  //  check sub-type
  if (!validateMsg(msg.stamp)) {return false;}
  // msg.frame_id :  @todo any check for it ?
  return true;
}

bool validateMsg(const geometry_msgs::msg::Point & msg)
{
  // @todo any check for following item ?
  msg.x;
  msg.y;
  msg.z;
  return true;
}
bool validateMsg(const geometry_msgs::msg::Quaternion & msg)
{
  // @todo any check for following item ?
  msg.x;
  msg.y;
  msg.z;
  msg.w;
  return true;
}

bool validateMsg(const geometry_msgs::msg::Pose & msg)
{
  // check sub-type
  if (!validateMsg(msg.position)) {return false;}
  if (!validateMsg(msg.orientation)) {return false;}
  return true;
}


// 验证地图元信息的函数
bool validateMsg(const nav_msgs::msg::MapMetaData & msg)
{
  // check sub-type
  if (!validateMsg(msg.map_load_time)) {return false;}
  if (!validateMsg(msg.origin)) {return false;}

  // @todo  any check for following item ?
  msg.height;
  msg.width;
  msg.resolution;


  return true;

}

// for msg-type like map, costmap and others as `OccupancyGrid`
bool validateMsg(const nav_msgs::msg::OccupancyGrid & msg)
{
  // check sub-type
  if (validateMsg(msg.header)) {return false;}
  // msg.data :  @todo any check for it ?
  if (validateMsg(msg.info)) {return false;}

  // check logic
  if (msg.data.size() != msg.info.width * msg.info.height) {false;}  // check map-size

  return true;
}


}


#endif
