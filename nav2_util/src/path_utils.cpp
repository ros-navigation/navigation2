/***********  nav2_util/path_utils.cpp  ***********/
#include <cmath>
#include <limits>
#include <algorithm>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

namespace
{
// ---------- helpers (XY-plane, no std::sqrt inside) ----------
inline double distanceSquared(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

inline double distanceSquaredToSegmentXY(
  const geometry_msgs::msg::Point & p,          // test point
  const geometry_msgs::msg::Point & a,          // segment start
  const geometry_msgs::msg::Point & b)          // segment end
{
  const double seg_len_sq = distanceSquared(a, b);
  if (seg_len_sq <= 1e-8) {        // degenerate segment
    return distanceSquared(p, a);
  }

  const double dot =
    ( (p.x - a.x) * (b.x - a.x) ) +
    ( (p.y - a.y) * (b.y - a.y) );
  const double t = std::clamp(dot / seg_len_sq, 0.0, 1.0);

  const double proj_x = a.x + t * (b.x - a.x);
  const double proj_y = a.y + t * (b.y - a.y);

  const double dx = p.x - proj_x;
  const double dy = p.y - proj_y;
  return dx * dx + dy * dy;
}

inline bool pathIsClosed(const nav_msgs::msg::Path & path, const double eps = 0.01)
{
  return path.poses.size() > 2 &&
         distanceSquared(path.poses.front().pose.position,
                         path.poses.back().pose.position) < eps * eps;
}
} // anonymous namespace

namespace nav2_util
{

/**
 *  Compute the shortest lateral (XY) distance from @p pose to @p path.
 *  If @p closest_idx is supplied the search starts there and the new index
 *  is written back for the next call, giving stable behaviour on loops.
 *
 *  Units: metres.
 *  Returns +∞ for an empty path.
 */
double distanceFromPath(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path,
  size_t * closest_idx /* = nullptr */)
{
  using std::numeric_limits;

  // -------- edge cases --------
  if (path.poses.empty()) {
    return numeric_limits<double>::infinity();
  }

  if (path.poses.size() == 1) {
    return std::hypot(
      pose.pose.position.x - path.poses.front().pose.position.x,
      pose.pose.position.y - path.poses.front().pose.position.y);
  }

  // -------- search setup --------
  const size_t n = path.poses.size();
  size_t start = 0;
  if (closest_idx && *closest_idx < n - 1) {    // valid previous index
    start = *closest_idx;
  }

  double best_dist_sq = numeric_limits<double>::max();
  size_t best_idx = start;

  // -------- scan segments i … n-2 --------
  for (size_t i = start; i < n - 1; ++i) {
    const double d_sq = distanceSquaredToSegmentXY(
      pose.pose.position,
      path.poses[i].pose.position,
      path.poses[i + 1].pose.position);

    if (d_sq < best_dist_sq) {
      best_dist_sq = d_sq;
      best_idx = i;
    }
  }

  // -------- handle seam of closed loop --------
  if (pathIsClosed(path)) {
    const double d_sq = distanceSquaredToSegmentXY(
      pose.pose.position,
      path.poses.back().pose.position,
      path.poses.front().pose.position);

    if (d_sq < best_dist_sq) {
      best_dist_sq = d_sq;
      best_idx = n - 1;
    }
  }

  // -------- update caller’s index --------
  if (closest_idx) {
    *closest_idx = (best_idx >= n - 1) ? 0 : best_idx;
  }

  return std::sqrt(best_dist_sq);
}

} // namespace nav2_util
