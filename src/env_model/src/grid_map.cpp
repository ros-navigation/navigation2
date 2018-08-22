#include "env_model/grid_map.h"
#include "ros/ros.h"
#include <math.h>    
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
//For now, implement calculations as previous
        

// MAP SPECIFIC //


// INTERFACE FUNCTIONS //

// get map coordinates from cell index
double GridMapEnv::getPosX(int i) {}

double GridMapEnv::getPosY(int j) {}

// get map cell index from map coordinates
int GridMapEnv::getCellX(double x) {}

int GridMapEnv::getCellY(double y) {}



//Check if map is valid 
bool GridMapEnv::isValid(int i, int j) {}

//get map value by index
float GridMapEnv::getValueAt(std::string layer, int i, int j) 
{
   
}

// get map value by position 
float GridMapEnv::getValueAtPos(std::string layer, double x, double y) 
{
    

}

bool GridMapEnv::getMap(std::string mapname)
{
    if (ros::service::call(mapname,map_msg))
    {
        ROS_INFO("GridMap Environment Created!");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    grid_map::GridMapRosConverter::fromMessage(map_msg.response.map,map);

    return 0;

}

bool GridMapEnv::initializeMapFromServer(std::string layer, std::string mapname)
{
   
}

void GridMapEnv::addMapLayer(std::string layer) 
{


}

void GridMapEnv::addMapLayer(std::string layer, std::string mapname) 
{

}
//Update the map layer specified
void GridMapEnv::updateMap(std::string layer) {}
        

// perform raytracing
double GridMapEnv::getRayTrace(std::string occ_layer, double ox, double oy, double oa, double max_range) 
{
   

}

void GridMapEnv::setLayerData(std::string layer){}

float* GridMapEnv::getMapLayer(std::string layer){}

int GridMapEnv::getMapSize(std::string layer){}

nav_msgs::MapMetaData GridMapEnv::getMapInfo(std::string layer){}