#include "map_server/map_reps/map_reps.h"
//#include "ros/ros.h"
//#include "ros/console.h"
#include "rclcpp/rclcpp.hpp"
//#include "nav_msgs/GetMap.h"
#include "nav_msgs/srv/get_map.hpp"

//#include <grid_map_msgs/GetGridMap.h>
//#include "map_server/map_converter.h"
//#include <typeinfo>
//#include <typeindex>
//#include <unordered_map>
#include <string>
#include "map_server/map_loader.h"


class MappingServerROS
{
    public:

        MappingServerROS(const std::string& fname, const std::string& map_type);

        rclcpp::Node::SharedPtr getNode(){return n;};
    private:

        MapLoader *m;
        BaseMapLoader* MyMap;
        //MapConverter Converter;
        //std::unordered_map<std::type_index, std::string> type_names;       
        
        rclcpp::Node::SharedPtr n;
        //ros::NodeHandle n;   
        


};