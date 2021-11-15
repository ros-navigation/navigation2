#ifndef NAV2_LOCALIZATION__SOLVER__MCLSolver_HPP_
#define NAV2_LOCALIZATION__SOLVER__MCLSolver_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav2_localization/plugins/solvers/particle_filter_base.hpp"

namespace nav2_localization
{

class MCLSolver : public ParticleFilterSolver
{
public:
  MCLSolver();

  /**
  * @brief
  * MCLSolver implementation
  */
  geometry_msgs::msg::PoseWithCovarianceStamped estimatePose(
    const nav_msgs::msg::Odometry & curr_odom,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) override;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__SOLVER__MCLSolver_HPP_
