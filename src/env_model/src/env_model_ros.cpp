#include "base_env_model.h"

#include "rclcpp/rclcpp.hpp"

#include "env_model_ros.h"

#include "env_model_msgs/srv/float_to_int.hpp"
#include "env_model_msgs/srv/int_to_float.hpp"
#include "env_model_msgs/srv/get_value_at.hpp"
#include "env_model_msgs/srv/get_value_at_pos.hpp"
#include "env_model_msgs/srv/is_map_valid.hpp"
#include "env_model_msgs/srv/get_ray_trace.hpp"
#include "env_model_msgs/srv/get_map.hpp"
        
EnvModelROS::EnvModelROS()
{   


    n = rclcpp::Node::make_shared("env_model");

    // User defined variables (TODO: pass from command line)
    map_type = "occupancy";
    base_layer = "static";
    base_map_srv = "static_occ_grid";


    RCLCPP_INFO(n->get_logger(),"Start Environment Model...");

    // Factory Pattern
    m = new EnvModelLoader();
    
    // Create appropriate map object
    MyEnv = m->createEnv(map_type,n);

    RCLCPP_INFO(n->get_logger(),"Environment object created...");

    //Initialize Map from Server into layer (default as static)
    MyEnv->initializeMapFromServer(base_layer, base_map_srv);

    RCLCPP_INFO(n->get_logger(),"Environment Map initialized...");



    // Create Services //

    auto handle_getPosCallback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<env_model_msgs::srv::IntToFloat::Request> request,
        std::shared_ptr<env_model_msgs::srv::IntToFloat::Response> response) -> void
    {
        (void)request_header;
        getPosCallback(request, response);
    };

    auto handle_getCellIndexCallback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<env_model_msgs::srv::FloatToInt::Request> request,
        std::shared_ptr<env_model_msgs::srv::FloatToInt::Response> response) -> void
    {
        (void)request_header;
        getCellIndexCallback(request, response);
    };

    auto handle_isMapValidCallback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<env_model_msgs::srv::IsMapValid::Request> request,
        std::shared_ptr<env_model_msgs::srv::IsMapValid::Response> response) -> void
    {
        (void)request_header;
        isMapValidCallback(request, response);
    };

    auto handle_getValueAtCallback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<env_model_msgs::srv::GetValueAt::Request> request,
        std::shared_ptr<env_model_msgs::srv::GetValueAt::Response> response) -> void
    {
        (void)request_header;
        getValueAtCallback(request, response);
    };

    auto handle_getValueAtPosCallback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<env_model_msgs::srv::GetValueAtPos::Request> request,
        std::shared_ptr<env_model_msgs::srv::GetValueAtPos::Response> response) -> void
    {
        (void)request_header;
        getValueAtPosCallback(request, response);
    };

    auto handle_getRayTraceCallback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<env_model_msgs::srv::GetRayTrace::Request> request,
        std::shared_ptr<env_model_msgs::srv::GetRayTrace::Response> response) -> void
    {
        (void)request_header;
        getRayTraceCallback(request, response);
    };

    auto handle_getMapCallback = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<env_model_msgs::srv::GetMap::Request> request,
        std::shared_ptr<env_model_msgs::srv::GetMap::Response> response) -> void
    {
        (void)request_header;
        getMapCallback(request, response);
    };



    getPos_srv = n->create_service<env_model_msgs::srv::IntToFloat>("getPos",handle_getPosCallback);
    getCell_srv = n->create_service<env_model_msgs::srv::FloatToInt>("getCell",handle_getCellIndexCallback);
    getRayTrace_srv = n->create_service<env_model_msgs::srv::GetRayTrace>("rayTrace",handle_getRayTraceCallback);
    isValid_srv = n->create_service<env_model_msgs::srv::IsMapValid>("isValid",handle_isMapValidCallback);
    getValueAt_srv = n->create_service<env_model_msgs::srv::GetValueAt>("getValueAt",handle_getValueAtCallback);
    getValueAtPos_srv = n->create_service<env_model_msgs::srv::GetValueAtPos>("getValueAtPos",handle_getValueAtPosCallback);
    getMap_srv = n->create_service<env_model_msgs::srv::GetMap>("getMap",handle_getMapCallback);


    rclcpp::spin(n);
}

    
bool EnvModelROS::getPosCallback(
    const std::shared_ptr<env_model_msgs::srv::IntToFloat::Request> req,
    const std::shared_ptr<env_model_msgs::srv::IntToFloat::Response> res)
{
    
    double x, y;
    x = MyEnv->getPosX(req->i[0]);
    y = MyEnv->getPosY(req->i[1]);
    std::vector<double> pos = {x, y};
    res->d  = pos;
    
    return true;
}

bool EnvModelROS::getCellIndexCallback(
    const std::shared_ptr<env_model_msgs::srv::FloatToInt::Request> req,
    const std::shared_ptr<env_model_msgs::srv::FloatToInt::Response> res)
{
    
    int i, j;
    i = MyEnv->getCellX(req->d[0]);
    j = MyEnv->getCellY(req->d[1]);
    std::vector<long int> index = {i, j};
    res->i  = index;
    
    return true;
}
bool EnvModelROS::isMapValidCallback(
    const std::shared_ptr<env_model_msgs::srv::IsMapValid::Request> req,
    const std::shared_ptr<env_model_msgs::srv::IsMapValid::Response> res)
{
    res->isvalid = MyEnv->isValid(req->index[0],req->index[1]);
    return true;
}  

bool EnvModelROS::getValueAtCallback(
    const std::shared_ptr<env_model_msgs::srv::GetValueAt::Request> req,
    const std::shared_ptr<env_model_msgs::srv::GetValueAt::Response> res)
{

    res->value = MyEnv->getValueAt(req->layer,req->index[0],req->index[1]);
    return true;
}   


bool EnvModelROS::getValueAtPosCallback(
    const std::shared_ptr<env_model_msgs::srv::GetValueAtPos::Request> req,
    const std::shared_ptr<env_model_msgs::srv::GetValueAtPos::Response> res)
{
   
    res->value = MyEnv->getValueAtPos(req->layer,req->pos[0],req->pos[1]);
    return true;
}

bool EnvModelROS::getRayTraceCallback(
    const std::shared_ptr<env_model_msgs::srv::GetRayTrace::Request> req,
    const std::shared_ptr<env_model_msgs::srv::GetRayTrace::Response> res)
{
    res->range = MyEnv->getRayTrace(base_layer,req->x, req->y, req->a, req->max);
    return true;
     
}

bool EnvModelROS::getMapCallback(
    const std::shared_ptr<env_model_msgs::srv::GetMap::Request> req,
    const std::shared_ptr<env_model_msgs::srv::GetMap::Response> res)
{

    float* values = MyEnv->getMapLayer(req->layer);
    int arrlen = MyEnv->getMapSize(req->layer);
    std::vector<double> mymap(values, values + arrlen);
    res->d = mymap;
    res->info = MyEnv->getMapInfo(req->layer);
    return true;
}



int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    try
    {
        EnvModelROS em;
        
    }
    catch(std::runtime_error& e)
    {
        //ROS_ERROR("env_model exception: %s", e.what());
        return -1;
    }

    return 0;
}        
      
