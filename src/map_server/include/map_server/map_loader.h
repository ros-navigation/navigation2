#ifndef MAP_SERVER_MAP_LOADER_H
#define MAP_SERVER_MAP_LOADER_H

#include "map_server/map_reps/map_reps.h"
//#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"


class MapLoader 
{
    public:
        //BaseMapLoader* createMap(std::string mapType,ros::NodeHandle n, std::string filename);
        BaseMapLoader* createMap(std::string mapType, rclcpp::Node::SharedPtr n, std::string filename);
        BaseMapLoader* createMap(std::string mapType);

};

#endif