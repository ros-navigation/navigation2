#ifndef MAP_SERVER_MAP_REPS_OCCUPANCY_GRID_LOADER_H
#define MAP_SERVER_MAP_REPS_OCCUPANCY_GRID_LOADER_H

#include "map_server/base_map_loader.h"
//#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

//#include "nav_msgs/GetMap.h"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

//#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
//#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "yaml-cpp/yaml.h"

//using namespace nav_msgs;

class OccGridLoader: public BaseMapLoader
{
    public:

    
        // Interface //

        void loadMapInfoFromFile(std::string fname, rclcpp::Node::SharedPtr n);
        
        void loadMapFromFile(std::string mapfname);
        

        void publishMap(){publishOccMap(map_msg);}

        void setMap(){setOccResponse(map_msg);}


        //void connectROS(ros::NodeHandle n){createROSInterface(n);}
        void connectROS(rclcpp::Node::SharedPtr n){createROSInterface(n);}

        //void createROSInterface(ros::NodeHandle n);
        void createROSInterface(rclcpp::Node::SharedPtr n);


        void publishOccMap(nav_msgs::msg::OccupancyGrid map);

        // Occupancy Grid Specific 
        
        nav_msgs::msg::OccupancyGrid map_msg;

        OccGridLoader(){}
        OccGridLoader(rclcpp::Node::SharedPtr n, std::string filename);
       
        ~OccGridLoader(){}

        void setOccResponse(nav_msgs::msg::OccupancyGrid map);

        void OccMapCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
            const std::shared_ptr<nav_msgs::srv::GetMap::Response> res);


    private:

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occmap_pub;
        rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service;

        //ros::Publisher occmap_pub;
        //ros::ServiceServer occ_service;
        nav_msgs::srv::GetMap::Response occmap_resp_;
};

#endif