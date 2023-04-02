#include <nav2_route/collision_checker.hpp> 

namespace nav2_route 
{
CollisionChecker::CollisionChecker(nav2_costmap_2d::Costmap2D *)
{
}

bool CollisionChecker::inCollision(const unsigned int & , const bool )
{
  return false; 
}
}
