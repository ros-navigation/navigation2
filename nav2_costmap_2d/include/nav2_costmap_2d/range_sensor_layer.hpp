// Copyright 2018 David V. Lu!!
// Ported to ROS2 by Michael Equi
#ifndef NAV2_COSTMAP_2D__RANGE_SENSOR_LAYER_HPP_
#define NAV2_COSTMAP_2D__RANGE_SENSOR_LAYER_HPP_

#include <list>
#include <string>
#include <vector>
#include <mutex>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/range.hpp"

namespace nav2_costmap_2d
{

class RangeSensorLayer : public CostmapLayer
{
public:
  enum InputSensorType
  {
    VARIABLE,
    FIXED,
    ALL
  };

  RangeSensorLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts( \
    nav2_costmap_2d::Costmap2D & master_grid, int min_i,
    int min_j, int max_i, int max_j);
  virtual void reset();
  virtual void deactivate();
  virtual void activate();

private:
  void bufferIncomingRangeMsg(const sensor_msgs::msg::Range::SharedPtr range_message);
  void processRangeMsg(sensor_msgs::msg::Range & range_message);
  void processFixedRangeMsg(sensor_msgs::msg::Range & range_message);
  void processVariableRangeMsg(sensor_msgs::msg::Range & range_message);

  void resetRange();
  void updateCostmap();
  void updateCostmap(sensor_msgs::msg::Range & range_message, bool clear_sensor_cone);

  double gamma(double theta);
  double delta(double phi);
  double sensor_model(double r, double phi, double theta);

  void get_deltas(double angle, double * dx, double * dy);
  void update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear);

  double to_prob(unsigned char c)
  {
    return static_cast<double>(c) / nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  unsigned char to_cost(double p)
  {
    return static_cast<unsigned char>(p * nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  std::function<void(sensor_msgs::msg::Range & range_message)> processRangeMessageFunc_;
  std::mutex range_message_mutex_;
  std::list<sensor_msgs::msg::Range> range_msgs_buffer_;

  double max_angle_, phi_v_;
  double inflate_cone_;
  std::string global_frame_;

  double clear_threshold_, mark_threshold_;
  bool clear_on_max_reading_;

  double no_readings_timeout_;
  rclcpp::Time last_reading_time_;
  unsigned int buffered_readings_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  double min_x_, min_y_, max_x_, max_y_;

  float area(int x1, int y1, int x2, int y2, int x3, int y3)
  {
    return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
  }

  int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy)
  {
    return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
  }
}
}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__RANGE_SENSOR_LAYER_HPP_
