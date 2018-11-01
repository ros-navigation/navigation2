cmake_minimum_required(VERSION 2.8)

include_directories( ../include )

add_library(crossdoor_nodes SHARED crossdoor_nodes.cpp )
add_library(dummy_nodes     SHARED dummy_nodes.cpp )
add_library(movebase_node   SHARED movebase_node.cpp )
