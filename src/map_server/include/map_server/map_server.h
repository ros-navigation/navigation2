#include "map_server/map_reps/map_reps.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "map_server/map_loader.h"


class MappingServerROS
{
    public:

        MappingServerROS(const std::string& fname, const std::string& map_type);

        //rclcpp::Node::SharedPtr getNode(){return n;};
    private:

        MapLoader *m;
        BaseMapLoader* MyMap;

        // TODO: Add converter for map representations
        // MapConverter Converter; 
        
        rclcpp::Node::SharedPtr n;
        


};