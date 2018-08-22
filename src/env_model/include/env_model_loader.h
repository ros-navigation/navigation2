#ifndef ENV_MODEL_ENV_MODEL_LOADER_H
#define ENV_MODEL_ENV_MODEL_LOADER_H


#include "env_models.h"
#include "rclcpp/rclcpp.hpp"

class EnvModelLoader 
{
    public:
        static BaseEnvModel* createEnv(std::string EnvType, rclcpp::Node::SharedPtr n);
};

#endif