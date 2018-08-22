#ifndef ENV_MODEL_GRID_MAP_ENV_H
#define ENV_MODEL_GRID_MAP_ENV_H

#include "env_model/base_env_model.h"
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GetGridMap.h>

class GridMapEnv : public BaseEnvModel
{


    public:
        
        
        GridMapEnv(){}

        // MAP SPECIFIC //
        grid_map::GridMap map;
        grid_map_msgs::GetGridMap map_msg;

        // INTERFACE FUNCTIONS //

        // get map coordinates from cell index
        double getPosX(int i);
        double getPosY(int j);
        // get map cell index from map coordinates
        int getCellX(double x);
        int getCellY(double y);

        // perform raytracing
        double getRayTrace(std::string layer, double x, double y, double a, double max_range);

        //Check if map is valid 
        bool isValid(int i, int j);

        //get map value by index
        float getValueAt(std::string layer, int i, int j);

        // get map value by position 
        float getValueAtPos(std::string layer, double x, double y);
        
        //Update the map layer specified
        void updateMap(std::string layer);
        
        bool getMap(std::string mapname);

        void addMapLayer(std::string layer);

        void addMapLayer(std::string layer,std::string mapname);

        bool initializeMapFromServer(std::string layer, std::string mapname);

        void setLayerData(std::string layer);
        
        float* getMapLayer(std::string layer);

        int getMapSize(std::string layer);

        nav_msgs::MapMetaData getMapInfo(std::string layer);
        ~GridMapEnv(){}


};

#endif