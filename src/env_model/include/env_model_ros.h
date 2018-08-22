#ifndef ENV_MODEL_ENV_MODEL_ROS_H
#define ENV_MODEL_ENV_MODEL_ROS_H

#include "base_env_model.h"

#include "rclcpp/rclcpp.hpp"

#include "env_model_msgs/srv/float_to_int.hpp"
#include "env_model_msgs/srv/int_to_float.hpp"
#include "env_model_msgs/srv/get_value_at.hpp"
#include "env_model_msgs/srv/get_value_at_pos.hpp"
#include "env_model_msgs/srv/is_map_valid.hpp"
#include "env_model_msgs/srv/get_ray_trace.hpp"
#include "env_model_msgs/srv/get_map.hpp"

#include "env_model_loader.h"

class EnvModelROS 
{


    public:
        
        
        EnvModelROS();

        ~EnvModelROS(){};
    private:
        
        rclcpp::Node::SharedPtr n;
  
        rclcpp::Service<env_model_msgs::srv::IntToFloat>::SharedPtr getPos_srv;
        rclcpp::Service<env_model_msgs::srv::FloatToInt>::SharedPtr getCell_srv;
        rclcpp::Service<env_model_msgs::srv::GetRayTrace>::SharedPtr getRayTrace_srv;
        rclcpp::Service<env_model_msgs::srv::IsMapValid>::SharedPtr isValid_srv;
        rclcpp::Service<env_model_msgs::srv::GetValueAt>::SharedPtr getValueAt_srv;
        rclcpp::Service<env_model_msgs::srv::GetValueAtPos>::SharedPtr getValueAtPos_srv;
        rclcpp::Service<env_model_msgs::srv::GetMap>::SharedPtr getMap_srv;

        
        std::string base_layer;
        std::string base_map_srv;
        std::string map_type;
        
        EnvModelLoader *m;
        BaseEnvModel* MyEnv;



        bool getPosCallback(
            const std::shared_ptr<env_model_msgs::srv::IntToFloat::Request> req,
            const std::shared_ptr<env_model_msgs::srv::IntToFloat::Response> res);

        bool getCellIndexCallback(
            const std::shared_ptr<env_model_msgs::srv::FloatToInt::Request> req,
            const std::shared_ptr<env_model_msgs::srv::FloatToInt::Response> res);        

        bool isMapValidCallback(
            const std::shared_ptr<env_model_msgs::srv::IsMapValid::Request> req,
            const std::shared_ptr<env_model_msgs::srv::IsMapValid::Response> res);  

        bool getValueAtCallback(
            const std::shared_ptr<env_model_msgs::srv::GetValueAt::Request> req,
            const std::shared_ptr<env_model_msgs::srv::GetValueAt::Response> res);   

        bool getValueAtPosCallback(
            const std::shared_ptr<env_model_msgs::srv::GetValueAtPos::Request> req,
            const std::shared_ptr<env_model_msgs::srv::GetValueAtPos::Response> res);

        bool getRayTraceCallback(
            const std::shared_ptr<env_model_msgs::srv::GetRayTrace::Request> req,
            const std::shared_ptr<env_model_msgs::srv::GetRayTrace::Response> res);           

        bool getMapCallback(
            const std::shared_ptr<env_model_msgs::srv::GetMap::Request> req,
            const std::shared_ptr<env_model_msgs::srv::GetMap::Response> res);                                                                                                
};



#endif