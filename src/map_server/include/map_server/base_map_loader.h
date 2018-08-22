#ifndef MAP_SERVER_BASE_MAP_LOADER_H
#define MAP_SERVER_BASE_MAP_LOADER_H

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
//#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"


enum MapMode {TRINARY, SCALE, RAW};

class BaseMapLoader
{


    public:
        

        // MetaData from YAML file, should this be defined here?  //
        
        std::string fname = "";
        std::string mapfname = "";
        double origin[3];
        int negate;
        double occ_th, free_th;
        double res;
        MapMode mode = TRINARY;
        std::string frame_id = "map";
        
        BaseMapLoader(){}
        
        virtual void loadMapInfoFromFile(std::string fname,rclcpp::Node::SharedPtr n)= 0;
        
        virtual void loadMapFromFile(std::string mapfname) = 0;

        virtual void publishMap() = 0;

        virtual void setMap() = 0;

        //virtual void connectROS(ros::NodeHandle n) = 0;
        virtual void connectROS(rclcpp::Node::SharedPtr n) = 0;
        
        ~BaseMapLoader(){}

};

#endif