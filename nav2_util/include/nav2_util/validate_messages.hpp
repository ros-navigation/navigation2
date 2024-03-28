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
//     1> Value Check: to avoid damaged value like like `nan`, `INF`, empty string and so on
//     2> Logic Check: to avoid value with bad logic, like the size of `map` should be equal to `height*width`
//     3> Any other needed condition could be joint here in future

#ifndef validate_message_HPP
#define validate_message_HPP


#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>

namespace nav2_util
{


bool validateMsg(const double & num)
{
  /*  @brief double/float value check
   *  if here'a need to check message validation
   *  it should be avoid to use double value like `nan`, `inf`
   *  otherwise, we regard it as an invalid message
   */
  if (std::isinf(num)) {return false;}
  if (std::isnan(num)) {return false;}
  return true;
}

const int NSEC_PER_SEC = 1e9;  // 1 second = 1e9 nanosecond
bool validateMsg(const builtin_interfaces::msg::Time & msg)
{
  /*  @brief time-stamp check
   *  if here'a need to check message validation
   *  it should at least have a non-zero time-stamp
   *  otherwise, we regard it as an invalid message
   */
  if (msg.sec == 0 && msg.nanosec == 0) {return false;}
  if (msg.nanosec >= NSEC_PER_SEC) {
    return false;                                      // invalid nanosec-stamp
  }
  return true;
}

bool validateMsg(const std_msgs::msg::Header & msg)
{
  //  check sub-type
  if (!validateMsg(msg.stamp)) {return false;}

  /*  @brief frame_id check
   *  if here'a need to check message validation
   *  it should at least have a non-empty frame_id
   *  otherwise, we regard it as an invalid message
   */
  if (msg.frame_id.empty()) {return false;}
  return true;
}

bool validateMsg(const geometry_msgs::msg::Point & msg)
{
  //  check sub-type
  if (!validateMsg(msg.x)) {return false;}
  if (!validateMsg(msg.y)) {return false;}
  if (!validateMsg(msg.z)) {return false;}
  return true;
}

const double epsilon = 1e-6;
bool validateMsg(const geometry_msgs::msg::Quaternion & msg)
{
  //  check sub-type
  if (!validateMsg(msg.x)) {return false;}
  if (!validateMsg(msg.y)) {return false;}
  if (!validateMsg(msg.z)) {return false;}
  if (!validateMsg(msg.w)) {return false;}

  // logic check
  // 1> the quaternion should be normalized:
  //     https://math.stackexchange.com/questions/1703466/normalizing-a-quaternion
  // @todo how to ensure that accuracy issue don't affect the judgment results ?
  // Here's a temporary method by setting Allowable Difference Range named as epsilon
  if (abs(msg.x * msg.x + msg.y * msg.y + msg.z * msg.z + msg.w * msg.w - 1.0) <= epsilon) {
    return false;
  }

  return true;
}

bool validateMsg(const geometry_msgs::msg::Pose & msg)
{
  // check sub-type
  if (!validateMsg(msg.position)) {return false;}
  if (!validateMsg(msg.orientation)) {return false;}
  return true;
}


// Function to verify map meta information
bool validateMsg(const nav_msgs::msg::MapMetaData & msg)
{
  // check sub-type
  if (!validateMsg(msg.origin)) {return false;}
  if (!validateMsg(msg.resolution)) {return false;}

  // logic check
  // 1> we don't need an empty map
  if (msg.height == 0 || msg.width == 0) {return false;}
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
  if (msg.data.size() != msg.info.width * msg.info.height) {
    return false;                                                          // check map-size

  }
  return true;
}


}


#endif
