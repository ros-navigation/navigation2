#include "occupancy_grid.h"
#include "rclcpp/rclcpp.hpp"
#include <math.h>    
#include <typeinfo>
#include "utilities/cacheddistmap.h"

        

// MAP SPECIFIC //

int OccupancyGridEnv::MapToDataIndex(int i, int j) {return i + j * size_x;}

nav_msgs::srv::GetMap::Response::SharedPtr OccupancyGridEnv::send_request(

    rclcpp::Node::SharedPtr node,
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client,
    nav_msgs::srv::GetMap::Request::SharedPtr request)
{
auto result = client->async_send_request(request);
// Wait for the result.
if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
{
    return result.get();
} else {
    return NULL;
    }

}


// INTERFACE FUNCTIONS //


void OccupancyGridEnv::addMapLayer(std::string layer) 
{   

    //ROS_INFO("Empty Layer '%s' added", layer.c_str());
    
}

void OccupancyGridEnv::addMapLayer(std::string layer, std::string mapname) 
{
    getMap(mapname);
    
    double res = map.info.resolution;
    int sx = map.info.width;
    int sy = map.info.height;
    double ox = map.info.origin.position.x + (sx / 2) * res;
    double oy = map.info.origin.position.y + (sy / 2) * res; 

    MyLayer *  new_layer = new MyData<decltype(map.data.data())>(layer, map.data.data(), sx, sy, res, ox, oy); // type: signed char*
    MyLayers[layer] = new_layer;
    //ROS_INFO("Layer '%s' added with map '%s'", layer.c_str(), mapname.c_str());
}


float* OccupancyGridEnv::getMapLayer(std::string layer)
{
    return MyLayers[layer]->get_data();
}

int OccupancyGridEnv::getMapSize(std::string layer){return MyLayers[layer]->get_size();}


nav_msgs::msg::MapMetaData OccupancyGridEnv::getMapInfo(std::string layer)
{ 
    
    return map.info;
}


// get map coordinates from cell index
double OccupancyGridEnv::getPosX(int i) {return origin_x + (i-size_x / 2) * resolution;}

double OccupancyGridEnv::getPosY(int j) {return origin_y + (j-size_y / 2) * resolution;}

// get map cell index from map coordinates
int OccupancyGridEnv::getCellX(double x) {return floor((x - origin_x) / resolution + 0.5) + size_x / 2;}

int OccupancyGridEnv::getCellY(double y) {return floor((y - origin_y) / resolution + 0.5) + size_y / 2;}



//Check if map is valid 
bool OccupancyGridEnv::isValid(int i, int j) {return (i >= 0) && (i < size_x) && (j >= 0) && (j < size_y);}

//get map value by index
float OccupancyGridEnv::getValueAt(std::string layer, int i, int j) 
{
    int index;
    float value;
    index = MapToDataIndex(i,j);

    value = (float) (MyLayers[layer]->get_value(index));

    return value;
}

// get map value by position 
float OccupancyGridEnv::getValueAtPos(std::string layer, double x, double y) 
{
    int i, j;
    i = getCellX(x);
    j = getCellY(y);
    return getValueAt(layer,i,j);

}

bool OccupancyGridEnv::getMap(std::string mapname)
{
    auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
    occ_client = n->create_client<nav_msgs::srv::GetMap>(mapname);
    auto response = send_request(n,occ_client,req);
    map = response->map;

    return 0;

}

bool OccupancyGridEnv::initializeMapFromServer(std::string layer, std::string mapname)
{   
    addMapLayer(layer, mapname); 
    // This is assumedly the master layer, so store metaInfo in class (not just layer)
    resolution = map.info.resolution;
    size_x = map.info.width;
    size_y = map.info.height;
    origin_x = map.info.origin.position.x + (size_x / 2) * resolution;
    origin_y = map.info.origin.position.y + (size_y / 2) * resolution;
    

    // How should we do something like this via interface -> i.e. create map layer from data (not map message) --> Boost::any? Call internal library? 
    CachedDistanceMap * cdm = new CachedDistanceMap(MyLayers[layer],resolution, 100.0, size_x, size_y);
    MyLayer * new_layer = new MyData<decltype(cdm->ccmap)>("distance", cdm->ccmap,size_x, size_y, resolution, origin_x, origin_y); // type: double*
    MyLayers["distance"] = new_layer;

    

    /*
    float* d = MyLayers["distance"]->get_data();
    std::vector<int> myints = {0,1,2,3};
    std::vector<int> myints2 = {100,101,102,103};
    MyLayers["distance"]->set_data(200,69.0);    
    MyLayers["distance"]->set_data(myints,MyLayers["distance"]->get_data(myints2)); 
    ROS_INFO("by index: %f , whole: %f",MyLayers["distance"]->get_value(100), MyLayers["distance"]->get_data(myints)[0]);  
    int size = MyLayers["distance"]->get_size();
    ROS_INFO("size: %u",size);
    */
   return 0;
}
void OccupancyGridEnv::setLayerData(std::string layer)
{
    //TODO: Generic interface for setting data into a layer NOT from a map. Generic data type? Call a utility function?

}



//Update the map layer specified
void OccupancyGridEnv::updateMap(std::string layer) 
{
}
        
// perform raytracing
double OccupancyGridEnv::getRayTrace(std::string occ_layer, double ox, double oy, double oa, double max_range) 
{
    // Bresenham raytracing
    int x0,x1,y0,y1;
    int x,y;
    
    int xstep, ystep;
    char steep;
    int tmp;
    int deltax, deltay, error, deltaerr;

    x0 = getCellX(ox);
    y0 = getCellY(oy);


    x1 = getCellX(ox + max_range * cos(oa));
    y1 = getCellY(oy + max_range * sin(oa));

    if(abs(y1-y0) > abs(x1-x0))
        steep = 1;
    else
        steep = 0;

    if(steep)
    {
        tmp = x0;
        x0 = y0;
        y0 = tmp;

        tmp = x1;
        x1 = y1;
        y1 = tmp;
    }

    deltax = abs(x1-x0);
    deltay = abs(y1-y0);
    error = 0;
    deltaerr = deltay;

    x = x0;
    y = y0;

    if(x0 < x1)
        xstep = 1;
    else
        xstep = -1;
    if(y0 < y1)
        ystep = 1;
    else
        ystep = -1;

    if(steep)
    {
    if(!isValid(y,x) || getValueAt(occ_layer,y,x) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }
    else
    {
    if(!isValid(x,y) || getValueAt(occ_layer,x,y) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }

    while(x != (x1 + xstep * 1))
    {
        x += xstep;
        error += deltaerr;
        if(2*error >= deltax)
        {
            y += ystep;
            error -= deltax;
        }

    if(steep)
    {
        if(!isValid(y,x) || getValueAt(occ_layer,y,x) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }
    else
    {
        if(!isValid(x,y) || getValueAt(occ_layer,x,y) > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * resolution;
    }
    }
    return max_range;

}
