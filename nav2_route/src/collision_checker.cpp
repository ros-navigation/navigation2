#include "nav2_route/collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include <iostream>

namespace nav2_route 
{
CollisionChecker::CollisionChecker(nav2_costmap_2d::Costmap2D *costmap)
{
  costmap_ = costmap;
}

bool CollisionChecker::inCollision(const unsigned int & i , const bool traverse_unknown)
{
  unsigned char cost = costmap_->getCost(i);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      (cost == nav2_costmap_2d::NO_INFORMATION && !traverse_unknown) ) {
    std::cout << "In collision" << std::endl;
    return true;
  }
  return false;
}
nav2_costmap_2d::Costmap2D *CollisionChecker::getCostmap() {
  return costmap_;
}

}  // namespace nav2_route