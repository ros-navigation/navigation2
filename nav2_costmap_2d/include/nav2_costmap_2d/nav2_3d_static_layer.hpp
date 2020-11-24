//
// Created by sun on 2020/10/10.
//

#ifndef NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP
#define NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_costmap_2d
{
// TODO 3D static layer now inherits from obstacle layer
// TODO but we have to discuss the overall design of 2D/3D data input and processing
class StaticLayer3D : public nav2_costmap_2d::CostmapLayer
{
  /*
   *
   */

public:
  StaticLayer3D();   // TODO a bit weird
  virtual ~StaticLayer3D();

// TODO Confirm: Is the 3 methods declaration necessary here?
// TODO Should 'override' keyword be placed here?
// TODO Best practice： void onInitialize() override;
// TODO What is exactly 'virtual' in C++?
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual bool receivedMap();
  virtual void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud);
  virtual void filteredPoints(sensor_msgs::msg::PointCloud2 pointcloud);
  virtual void fillCostMapFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

private:
  nav2_costmap_2d::Costmap2D _map_2d;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
  unsigned int _map_size_x;
  unsigned int _map_size_y;
  double _map_resolution;
// TODO Is the (origin_x, origin_y) a relative axis origin point
  double _origin_x;
  double _origin_y;
  double _lethal_threshold;
  std::string _topic_name;

  /*
   * voxel grid parameters
   */
  float _voxel_leafsize;    // TODO Should voxel_leafsize_ be double?
  double _min_z_height;
  double _max_z_height;

  bool map_received_;

};

}

#endif //NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP
