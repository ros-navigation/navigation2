#ifndef MAP_SERVER_BASE_MAP_LOADER_H
#define MAP_SERVER_BASE_MAP_LOADER_H

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include "rclcpp/rclcpp.hpp"



class BaseMapLoader
{

    public:
        
        
        std::string fname;
        std::string mapfname;

        BaseMapLoader(){}
        
        virtual void loadMapInfoFromFile(std::string fname)= 0;
        
        virtual void loadMapFromFile(std::string mapfname) = 0;

        virtual void publishMap() = 0;

        virtual void setMap() = 0;

        virtual void connectROS(rclcpp::Node::SharedPtr n) = 0;
        
        ~BaseMapLoader(){}

};

#endif