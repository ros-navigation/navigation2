//
// Created by shivam on 9/24/20.
//

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_map_server/map_server_core.hpp"
namespace nav2_map_server{

template<class mapT> MapServer<mapT>::MapServer() : nav2_util::LifecycleNode("map_server"){

}

template<class mapT> MapServer<mapT>::~MapServer() = default;

template<class mapT>
nav2_util::CallbackReturn
MapServer<mapT>::on_configure(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapServer<mapT>::on_activate(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapServer<mapT>::on_deactivate(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapServer<mapT>::on_cleanup(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapServer<mapT>::on_shutdown(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;

}

template<class mapT>
nav2_util::CallbackReturn
    MapServer<mapT>::on_error(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;

}
}
