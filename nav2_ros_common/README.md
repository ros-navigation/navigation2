# Nav2 ROS Common

This package contains common utilities and definitions used across the Nav2 stack in ROS 2.
These include the Nav2 base Lifecycle Node, Action Server, Service Server, Service Client, Node Thread, and other common components used in Nav2 and related packages.

The expectation is that all Nav2 nodes and packages use the utilities in this package globally to be consistent and to avoid code duplication.
The lifecycle node includes factories for subscriptions, publishers, clients, and servers that return Nav2 objects in this package rather than the ROS 2 base classes as a level of abstraction.

This is a header-only package.
