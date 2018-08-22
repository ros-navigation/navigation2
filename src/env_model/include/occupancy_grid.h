#ifndef ENV_MODEL_OCCUPANCY_GRID_ENV_H
#define ENV_MODEL_OCCUPANCY_GRID_ENV_H

#include "base_env_model.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include <unordered_map>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "occ_grid_layer.h"
#include <vector>


class OccupancyGridEnv : public BaseEnvModel
{
    private:
        double resolution;
        int size_x;
        int size_y;
        double origin_x;
        double origin_y;

        //This map is a placeholder, it can be overwritten via getMap 
        nav_msgs::msg::OccupancyGrid map;
        nav_msgs::srv::GetMap occ_srv; 
        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr occ_client;
        //nav_msgs::srv::GetMap::Request::SharedPtr occmap_req_;
        
        rclcpp::Node::SharedPtr n;
        //This is our container for map layers 

        std::unordered_map<std::string, MyLayer*> MyLayers;  
     



       

    public:
        
   
        OccupancyGridEnv(rclcpp::Node::SharedPtr node){n = node;}

        // MAP SPECIFIC //
        
        int MapToDataIndex(int i, int j);


        nav_msgs::srv::GetMap::Response::SharedPtr send_request(

            rclcpp::Node::SharedPtr node,
            rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client,
            nav_msgs::srv::GetMap::Request::SharedPtr request);
        





        // INTERFACE FUNCTIONS //

        // get map coordinates from cell index
        double getPosX(int i);
        double getPosY(int j);

        // get map cell index from map coordinates
        int getCellX(double x);
        int getCellY(double y);

        // perform raytracing
        double getRayTrace(std::string occ_layer, double ox, double oy, double oa, double max_range);

        //Check if map is valid 
        bool isValid(int i, int j);

        //get map value by index
        float getValueAt(std::string layer, int i, int j);

        // get map value by position 
        float getValueAtPos(std::string layer, double x, double y);
        
        // grab map from map server
        bool getMap(std::string mapname);

        // initialize metadata and layer from map
        bool initializeMapFromServer(std::string layer, std::string mapname);



        // add a specified map to layer (may decide to reject map if doesn't match the metadata)
        void addMapLayer(std::string layer,std::string mapname);

        float* getMapLayer(std::string layer);

        int getMapSize(std::string layer);

        nav_msgs::msg::MapMetaData getMapInfo(std::string layer);
        

        // TODO //

        void addMapLayer(std::string layer);

        void updateMap(std::string layer);
        
        void setLayerData(std::string layer);

        ~OccupancyGridEnv(){}



};

#endif