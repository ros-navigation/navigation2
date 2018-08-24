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
    else
    {
        fprintf(stderr,"[ERROR] [map_server]: Cannot Load Map of Type '%s'\n", mapType.c_str());
        exit(-1);
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

    else
    {
        fprintf(stderr,"[ERROR] [map_server]: Cannot Load Map of Type '%s'\n", mapType.c_str());
        exit(-1);
    }
}