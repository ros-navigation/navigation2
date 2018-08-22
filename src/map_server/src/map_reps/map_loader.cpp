#include "map_server/map_loader.h"

//Factory Class for loading new map types

BaseMapLoader* MapLoader::createMap(std::string mapType, rclcpp::Node::SharedPtr n, std::string filename)
{
    if (mapType=="occupancy")
    {
        return new OccGridLoader(n,filename);
    }
    
    else if (mapType=="gridmap")
    {

        return new OccGridLoader(n,filename);
    }
    
}

BaseMapLoader* MapLoader::createMap(std::string mapType)
{
    if (mapType=="occupancy")
    {

        return new OccGridLoader;
    }
     else if (mapType=="gridmap")
    {
        return new OccGridLoader;
    

    } 
}