#ifndef MAP_SERVER_MAP_REPS_GRIDMAP_LOADER_H
#define MAP_SERVER_MAP_REPS_GRIDMAP_LOADER_H

#include "map_server/base_map_loader.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

using namespace grid_map;

class GridMapLoader: public BaseMapLoader
{
    public:
        GridMap gridMap;
        grid_map_msgs::GridMap map_msg;

        //Map<grid_map_msgs::GridMap> map_msg;

        cv::Mat MapImage;
        
        GridMapLoader(){}
        
        void loadMapInfoFromFile(std::string fname);
        
        void loadMapFromFile(std::string mapfname);

        void publishMap(){}
        void setMap(){}
        void connectROS(ros::NodeHandle n){}
        ~GridMapLoader(){}
        

};

#endif