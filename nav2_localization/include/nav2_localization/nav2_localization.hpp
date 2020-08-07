#ifndef NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_
#define NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"

namespace nav2_localization
{

class LocalizationServer : public nav2_util::LifecycleNode
{
public:

    /**
    * @brief Constructor for nav2_localization::LocalizationServer
    */
    LocalizationServer();
    /**
    * @brief Destructor for nav2_localization::LocalizationServer
    */
    ~LocalizationServer();

protected:
    /**
     * @brief Configures server parameters and member variables
     *
     * Configures motion model and matcher plugins; Initialize odom subscriber.
     * @param state LifeCycle Node's state
     * @return Success or Failure
     * @throw pluginlib::PluginlibException When failed to initialize motion
     * model or matcher plugins
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Activates member variables
     *
     * Activates motion model and matcher.
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Deactivates member variables
     *
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Calls clean up states and resets member variables.
     *
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Called when in Shutdown state
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Called when in Error state
     * @param state LifeCycle Node's state
     * @return Success or Failure
     */
    nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

    // Publishers and subscribers
    std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
};

}

#endif // NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_