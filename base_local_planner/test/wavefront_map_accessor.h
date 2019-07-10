/*
 * wavefront_map_accessor.h
 *
 *  Created on: May 2, 2012
 *      Author: tkruse
 */

#ifndef WAVEFRONT_MAP_ACCESSOR_H_
#define WAVEFRONT_MAP_ACCESSOR_H_
#include <costmap_2d/cost_values.h>
namespace base_local_planner {

/**
 * Map_grids rely on costmaps to identify obstacles. We need a costmap that we can easily manipulate for unit tests.
 * This class has a grid map where we can set grid cell state, and a synchronize method to make the costmap match.
 */
class WavefrontMapAccessor : public costmap_2d::Costmap2D {
  public:
    WavefrontMapAccessor(MapGrid* map, double outer_radius)
      : costmap_2d::Costmap2D(map->size_x_, map->size_y_, 1, 0, 0),
        map_(map), outer_radius_(outer_radius) {
      synchronize();
    }

    virtual ~WavefrontMapAccessor(){};

    void synchronize(){
      // Write Cost Data from the map
      for(unsigned int x = 0; x < size_x_; x++){
        for (unsigned int y = 0; y < size_y_; y++){
          unsigned int ind = x + (y * size_x_);
          if(map_->operator ()(x, y).target_dist == 1) {
            costmap_[ind] = costmap_2d::LETHAL_OBSTACLE;
          } else {
            costmap_[ind] = 0;
          }
        }
      }
    }

  private:
    MapGrid* map_;
    double outer_radius_;
};

}



#endif /* WAVEFRONT_MAP_ACCESSOR_H_ */
