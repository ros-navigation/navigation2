#pragma once

// #include <stdio.h>
#include <cstdio>

// TODO: Figure out how to get a reference to the node logger
#define ROS_ERROR(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_WARN(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_INFO(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_DEBUG(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_DEBUG_NAMED(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_INFO_ONCE(...) std::fprintf(stderr, __VA_ARGS__)
