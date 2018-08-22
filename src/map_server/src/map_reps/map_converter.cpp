#include "map_server/map_converter.h"
#include <grid_map_core/grid_map_core.hpp>

using namespace grid_map;
boost::any MapConverter::convertMap(const nav_msgs::OccupancyGrid occ, GridMap grid,const std::string& layer)
{
    GridMapRosConverter::fromOccupancyGrid(occ, layer, grid);
    converted = grid;
    return converted;
}

boost::any MapConverter::convertMap(GridMap grid, nav_msgs::OccupancyGrid occ, const std::string& layer, 
                            double occ_thresh, double free_thresh)
{
    // dataMin, dataMax are hard set to [0,255] but this could be a parameter 
    dataMin = 0;
    dataMax = 255;
    
    Eigen::MatrixXf data;
    data = grid.get("occupancy");
    data = (data.array() < free_thresh*dataMax).select(dataMin, data);
    data = (data.array() > occ_thresh*dataMax).select(dataMax, data);
    data = (data.array() > free_thresh*dataMax && data.array() < occ_thresh*dataMax ).select(nanf(""),data);

    grid.add("occupancy",data);
    
    GridMapRosConverter::toOccupancyGrid(grid,layer,dataMin,dataMax, occ);
    converted = occ;
    return converted;
}