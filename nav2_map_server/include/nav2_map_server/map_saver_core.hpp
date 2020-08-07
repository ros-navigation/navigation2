//
// Created by shivam on 9/15/20.
//

#ifndef NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_MAP_SAVER_HPP_
#define NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_MAP_SAVER_HPP_

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_map_server{

template <class mapT>
class MapSaver : public nav2_util::LifecycleNode
{
 public:
  /**
   * @brief A constructor for nav2_map_server::MapServer
   */
  MapSaver();

  /**
   * @brief A Destructor for nav2_map_server::MapServer
   */
  ~MapSaver() override;

 protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Start publishing the map using the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Stops publishing the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when Error is raised
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

};
}  // namespace nav2_map_server
#endif //NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_MAP_SAVER_HPP_
