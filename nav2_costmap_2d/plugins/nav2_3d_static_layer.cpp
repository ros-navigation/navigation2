//
// Created by sun on 2020/10/10.
//

#include "nav2_costmap_2d/nav2_3d_static_layer.hpp"

#include "octomap/octomap.h"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_2d
{
StaticLayer3D::StaticLayer3D()
{
  _map_2d = nav2_costmap_2d::Costmap2D();
}
StaticLayer3D::~StaticLayer3D() {}

void
StaticLayer3D::onInitialize()
{
  RCLCPP_INFO(
    logger_,
    "humuhumunukunukuapuaa is loading 3d static map"
  );
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic_name", rclcpp::ParameterValue("pc2_map"));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(2.0));
  declareParameter("voxel_leafsize", rclcpp::ParameterValue(0.2));
  declareParameter("min_z_height", rclcpp::ParameterValue(0.0));
  declareParameter("max_z_height", rclcpp::ParameterValue(3.0));

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "topic_name", _topic_name);
  node->get_parameter(name_ + "." + "lethal_threshold", _lethal_threshold);
  node->get_parameter(name_ + "." + "voxel_leafsize", _voxel_leafsize);
  node->get_parameter(name_ + "." + "min_z_height", _min_z_height);
  node->get_parameter(name_ + "." + "max_z_height", _max_z_height);

  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = NO_INFORMATION;
  global_frame_ = layered_costmap_->getGlobalFrameID();
  map_received_ = false;
  /*
   * TODO:QoS part should be more specific
   * the observation function could be test when map server ready
   */
  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);
  std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_pc2(new sensor_msgs::msg::PointCloud2());

  _subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    _topic_name, map_qos,
    std::bind(&StaticLayer3D::cloudCallback, this, std::placeholders::_1));
}


void
StaticLayer3D::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if(!map_received_)
  {
      map_received_ = true;
      RCLCPP_INFO(
              logger_,
              "here"
              );
  }
  Costmap2D * master = layered_costmap_->getCostmap();
  _map_resolution = master->getResolution();
  _map_size_x = master->getSizeInMetersX() / _map_resolution;
  _map_size_y = master->getSizeInMetersY() / _map_resolution;
  _origin_x = master->getOriginX();
  _origin_y = master->getOriginY();

  _map_2d = nav2_costmap_2d::Costmap2D(
    _map_size_x,
    _map_size_y,
    _map_resolution,
    _origin_x,
    _origin_y);
  RCLCPP_INFO(
    logger_,
    "<<<<humuhumunukunukuapuaa>>>>created 2d map with size: %d X %d, resolution: %lf, origin: (%lf, %lf)",
    _map_size_x, _map_size_y, _map_resolution, origin_x_, origin_y_
  );

  // filteredPoints(*pointcloud);
  fillCostMapFromPointCloud(pointcloud);

}

void
StaticLayer3D::filteredPoints(sensor_msgs::msg::PointCloud2 cloud){
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  float scale = 0.1;
  for (; iter_x != iter_x.end();){
    for (; iter_y != iter_y.end(); ){
      if (*iter_z >= lethal_threshold_){
        unsigned int map_x = (unsigned int) (*iter_x) * scale;
        unsigned int map_y = (unsigned int) (*iter_y) * scale;
        if (map_2d_.getCost(map_x, map_y) != LETHAL_OBSTACLE){
          map_2d_.setCost(map_x, map_y, LETHAL_OBSTACLE);
        }
      }
      ++iter_z;
      ++iter_y;
      ++iter_x;
    }
  }
}

void
StaticLayer3D::fillCostMapFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud, *cloud_pcl);
  pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor_voxelgrid;
  sor_voxelgrid.setInputCloud(cloud_xyz);
  sor_voxelgrid.setLeafSize(_voxel_leafsize, _voxel_leafsize, _voxel_leafsize);
  sor_voxelgrid.setFilterFieldName("z");
  sor_voxelgrid.setFilterLimits(_min_z_height, _max_z_height);
  sor_voxelgrid.setFilterLimitsNegative(false);
  sor_voxelgrid.filter(*filtered_cloud);

  octomap::OcTree octotree(_map_resolution);

  for (auto p:filtered_cloud->points) {
    octotree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
  }
  octotree.updateInnerOccupancy();
  for (octomap::OcTree::iterator it = octotree.begin(), end = octotree.end();
    it != end; ++it)
  {
    if (octotree.isNodeOccupied(*it)) {
      auto coord = it.getCoordinate();
      /*
       *
       */
      // TODO CPP recommends static_cast<int>
      // TODO Be aware of the implicit accuracy loss
      // TODO Make it clear why the forum is like this:
      // Ref: https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/src/costmap_2d.cpp#L264
      int point_x = (int) (coord.x() - (origin_x_ - 0.5) / _map_resolution);
      int point_y = (int) (-(origin_y_ - 0.5) / _map_resolution - coord.y());
      int point_z = (int) coord.z();

      // TODO What does the implicit conversion means?
      unsigned int x = point_x;
      unsigned int y = point_y;
      // TODO More conditions to validate the result?
      if (x > _map_size_x || y > _map_size_y) {
        // TODO If the map is invalid, make a warning or error
        // then break
        RCLCPP_INFO(
          logger_,
          "<<<<humuhumunukunukuapuaa  the world map coord transformation>>>>over: x: %lf,y: %lf",
          x, y
        );
        break;
      } else {
        if (point_z > _lethal_threshold) {
          _map_2d.setCost(x, y, LETHAL_OBSTACLE);
        } else if (point_z < _lethal_threshold) {
          _map_2d.setCost(x, y, FREE_SPACE);
        }
      }
    }
  }
}

bool
StaticLayer3D::receivedMap()
{
  if (map_received_) {
    RCLCPP_INFO(
      logger_,
      "map has received"
    );
    return true;
  } else {
    RCLCPP_INFO(
      logger_,
      "NOT received"
    );
    return false;
  }
}

void
StaticLayer3D::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  RCLCPP_INFO(
    logger_,
    "updating Bounds"
  );
  RCLCPP_INFO(
    logger_,
    "show robot position: x: %lf, y:%lf ",
    robot_x, robot_y
  );
  useExtraBounds(min_x, min_y, max_x, max_y);
  /*
   *
   */
  *min_x = -(_map_size_x * _map_resolution);
  *min_y = -(_map_size_y * _map_resolution);
  *max_x = _map_size_x * _map_resolution;
  *max_y = _map_size_y * _map_resolution;


}

void
StaticLayer3D::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  RCLCPP_INFO(
    logger_,
    "updating Costs"
  );
  RCLCPP_INFO(
    logger_,
    "show cost bbox: %d %d  %d %d ",
    min_i, min_j, max_i, max_j
  );

  // TODO Only update the cost within the bounds
  for (unsigned int i = 0; i < _map_size_x; ++i) {
    for (unsigned int j = 0; j < _map_size_y; ++j) {
      master_grid.setCost(i, j, _map_2d.getCost(i, j));
    }
  }
}
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::StaticLayer3D, nav2_costmap_2d::Layer)
