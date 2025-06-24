# Nav2 ROS Common

This package contains common header-only utilities and definitions used across the Nav2 stack in ROS 2.
These include the Nav2 base Lifecycle Node, Action Server, Service Server, Service Client, Node Thread, and other common components used in Nav2 and related packages.

The expectation is that all Nav2 nodes and packages use the utilities in this package globally to be consistent and to avoid code duplication.
The lifecycle node includes factories for subscriptions, publishers, clients, and servers that return Nav2 objects in this package rather than the ROS 2 base classes as a level of abstraction of common boilerplate configurations and capabilities.
The factories are listed below:

- `create_client` --> `nav2::ServiceClient`
- `create_service` --> `nav2::ServiceServer`
- `create_publisher` --> `nav2::Publisher`
- `create_subscriber` --> `nav2::Subscriber`
- `create_action_server` --> `nav2::SimpleActionServer`
- `create_action_client` --> `nav2::ActionClient`

This is in some cases a big change from `rclcpp` versions, though most have direct analogs or are simply wrappers with additional configurations pre-baked in to remove tedious repetition on every package.
It also allows a central and controlled location to make future changes such as adding tracing, replacing APIs, controlling QoS profiles, or enabling advanced settings.
Thus, see the migration guide below for how to migrate from the previous versions of these objects to the new ones in this package from ROS 2 Kilted to Lyrical.

## Migration Guide

To see an example of migration, see the [migration example](https://github.com/ros-navigation/navigation2/pull/5288) of the Nav2 stack.

### General information

* `nav2_util::LifecycleNode` is now `nav2::LifecycleNode`, which is largely the same except for the factories and a couple of internal implementation details.
* The Service Client, Service Server, and Simple Action Server were also moved to the `nav2::` namespace, but they should not be accessed directly. Use the `create_*` factories from the `nav2::LifecycleNode`, such as `create_action_server` or `create_action_client`.
* There are now `nav2::qos` profiles for QoS to be used in the codebase for homologation and later easier changing: `nav2::qos::StandardTopicQoS` `nav2::qos::LatchedPublisherQoS`, `nav2::qos::LatchedSubscriberQoS` and `nav2::qos::SensorDataQoS`. These should be used rather than `rclcpp` profiles.
* The APIs for `create_*` are very similar to the default ones, but slightly different to move the now optional QoS profile specification below required information. When this is not specified the `StandardTopicQoS` is used (reliable, queue size 10). Only override this if you want another QoS type -- this is supposed to help make sure that things keep compatible and consistent. Avoid use of `SystemDefaultsQoS`.

### Plugin Migration

All plugins now use `nav2::LifecycleNode` instead of `rclcpp_lifecycle::LifecycleNode` or `nav2_util::LifecycleNode`. All must be updated to use this new API from planners and controllers to behavior tree nodes. Similarly, calls to the `create_*` factories will now point to the Nav2 objects, where applicable. See above for what those look like and below for a migration from the existing code usage to the new version in order to make use of the new features and API.

### Service Client Migration Example

We no longer need to create the object manually, nor should we as it bypasses the lifecycle node factories that set introspection and other future features. We can use the node now to do this instead of passing in a node and we don't need to specify the node type anymore as a template.
All Nav2 servers should use `nav2::ServiceClient<T>`.

```
// If using rclcpp::Client<T> from previous create_client factor
main_client_ = node->create_client<SrvT>(main_srv_name_, rclcpp::SystemDefaultsQoS(), callback_group_);  // Type rclcpp:Client<T>

// If using nav2_util::ServiceClient<T> manually
main_client_ =
  std::make_shared<nav2_util::ServiceClient<SrvT>>(
  main_srv_name_,
  node,
  false /* Does not create and spin an internal executor*/);  // Type nav2_util::ServiceClient<T>
```

To:

```
main_client_ = node->create_client<SrvT>(main_srv_name_, false  /* Does not create and spin an internal executor*/);  // Type nav2::ServiceClient<T>
```


### Service Server Migration Example

Services should now use `nav2::ServiceServer<T>` instead of `rclcpp::Service<T>` or `nav2_util::ServiceServer<T>`. The factory is now available from the node `create_service(...)`, so we can use that to create the service server.
The callback should now include the `rmw_request_id_t` header now, so we have 3 placeholders rather than 2:

```
// If using previous create_service factory
  service_ = node->create_service<std_srvs::srv::Trigger>(
    std::string(node->get_name()) + "/" + getName() + "/reroute",
    std::bind(
      &ReroutingService::serviceCb, this,
      std::placeholders::_1, std::placeholders::_2));  // type rclcpp::Service<T>

// If using nav2_util::ServiceServer<T> manually
  service_ = std::make_shared<nav2_util::ServiceServer<std_srvs::srv::Trigger,
      std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>>(
    std::string(node->get_name()) + "/" + getName() + "/reroute",
    node,
    std::bind(
      &ReroutingService::serviceCb, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));  // type nav2_util::ServiceServer<T>

```

To

```
  service_ = node->create_service<std_srvs::srv::Trigger>(
    std::string(node->get_name()) + "/" + getName() + "/reroute",
    std::bind(&ReroutingService::serviceCb, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));  // type nav2::ServiceServer<T>
```

### Action Server Migration

We can use the factory now from the node and the node is not required as an argument.
This otherwise does not change.
This is analog to the action server but configures with action introspection and other features that are not available in the base `rclcpp_action::Server<T>`.


```
  compute_and_track_route_server_ = std::make_shared<ComputeAndTrackRouteServer>(
    node, "compute_and_track_route",
    std::bind(&RouteServer::computeAndTrackRoute, this),
    nullptr, std::chrono::milliseconds(500), true);  // Type nav2_util::SimpleActionServer<T>
```

To

```
  compute_and_track_route_server_ = create_action_server<ComputeAndTrackRoute>(
    "compute_and_track_route",
    std::bind(&RouteServer::computeAndTrackRoute, this),
    nullptr, std::chrono::milliseconds(500), true);  // Type nav2::SimpleActionServer<T>
```

### Action Client Migration

We can use the node now to create an action client using `create_action_client` without providing all the node interfaces.
This is analog to the action client but configures with action introspection and other features that are not available in the base `rclcpp_action::Client<T>`.

```
  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose", callback_group_);  // Type rclcpp_action::Client<T>
```

To

```
  nav_to_pose_client_ = create_action_client<ClientT>(
    "navigate_to_pose", callback_group_);  // Type nav2::ActionClient<T>
```

### Publisher Subscriber Migration

To migrate, the order of the arguments in the Subscription must change since the QoS profile is now optional. It is now `(topic, callback, QoS)` whereas QoS defaults to the StandardTopicQoS, which is the same as `rclcpp::QoS` for the moment.

Publishers that explicitly specify a QoS profile do not require changes, though if the constructor using `depth` is used, it must now specify a policy explicitly. Both are now `nav2::Publisher` and `nav2::Subscription` objects that today just typedef the rclcpp and rclcpp_lifecycle versions. In the future, more features will be added here like lifecycle support for the subscriptions, so its highly recommended as part of this migration to migrate the `rclcpp::` to `nav2::` as well so those updates are already easily available.

```
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(), callback_group);

  data_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
    source_topic, range_qos,
    std::bind(&Range::dataCallback, this, std::placeholders::_1));
```

To

```
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan");  // No QoS is required if using the StandardTopicQoS, else it can be provided

  data_sub_ = node->create_subscription<sensor_msgs::msg::Range>(
    source_topic,
    std::bind(&Range::dataCallback, this, std::placeholders::_1));  // QoS can be omitted if using StandardTopicQoS, else it can be provided last
```
