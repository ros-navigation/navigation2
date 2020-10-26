//
// Created by sun on 2020/10/10.
//

#ifndef NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP
#define NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace nav2_3d_static_layer  // TODO rename ns to nav2_costmap_2d
{
// TODO remove the leading indentation inside ns
// TODO 3D static layer now inherits from obstacle layer
// TODO but we have to discuss the overall design of 2D/3D data input and processing
    class Nav23dStaticLayer : public nav2_costmap_2d::ObstacleLayer
    {
        /*
         *
         */
    public:
        Nav23dStaticLayer(); // TODO a bit weird
        virtual ~Nav23dStaticLayer();

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

        virtual void readPC(std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_pc2);

        virtual void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud);
// TODO Is a 'pass by reference' better for param cloud?
// TODO Confirm: sensor_msgs::msg::PointCloud2 looks like a complex structure
// TODO There are many different ways passing arguments here: ref, pointer, shared_ptr, dedicated ConstSharedPtr
// TODO If the type name is too long, use typedef or using instead
        virtual void convertTo2d(sensor_msgs::msg::PointCloud2 cloud);
    private:
// TODO putting _ after var name to indicate private member, is this a new fashion?
        nav2_costmap_2d::Costmap2D map_2d_;
        unsigned int map_size_x_;
        unsigned int map_size_y_;
        double map_resolution_;
// TODO Is the (origin_x, origin_y) a relative axis origin point
        double origin_x_;
        double origin_y_;
        /*
         *  mark_threshold_ : >> ?? <<
         */
        unsigned char lethal_threshold_;
        std::string topic_name_;

        /*
         * voxel grid parameters
         */
        float voxel_leafsize_;  // TODO Should voxel_leafsize_ be double?
        double min_z_height_;
        double max_z_height_;
    };

}

#endif //NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP
