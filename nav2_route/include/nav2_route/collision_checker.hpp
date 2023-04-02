#include <nav2_costmap_2d/costmap_2d.hpp> 


#ifndef NAV2_ROUTE__COLLISION_MONITOR_HPP_
#define NAV2_ROUTE__COLLISION_MONITOR_HPP_

namespace nav2_route {

class CollisionChecker 
{
public: 

  CollisionChecker(nav2_costmap_2d::Costmap2D * costmap); 
  
  bool inCollision(const unsigned int & i, const bool traverse_unknown); 
};
}  // namepsace nav2_route

#endif  // NAV2_ROUTE__COLLISION_MONITOR_HPP_