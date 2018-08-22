#include "env_model_loader.h"


BaseEnvModel* EnvModelLoader::createEnv(std::string EnvType,rclcpp::Node::SharedPtr n)
{
    if (EnvType=="occupancy")
    {

       return new OccupancyGridEnv(n);
    }
    /*
    else if (EnvType=="gridmap")
    {
        return new GridEnv;
        return GridEnv;
    }   
    */                        
}
