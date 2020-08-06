#ifndef NAV2_LOCALIZATION__MAP_HPP_
#define NAV2_LOCALIZATION__MAP_HPP_

#include <vector>

namespace nav2_localization
{
	// NDT Grid cell for NDT Grid Map
	typedef struct
	{   
	    float mean;
	    float covariance;
	} ndt_cell_t;
	
	
	// 2D NDT Grid Map for NDT matching
	typedef struct
	{   
	    std::vector<std::vector<NDTCell>>
	} ndt_grid_2d_t;

/*
	// 3D NDT Grid Map for NDT matching
	typedef struct
	{   
	    std::vector<std::vector<std::vector<NDTCell>>>
	} ndt_grid_3d_t;
*/
}

#endif // NAV2_LOCALIZATION__MAP_HPP_
