# Copyright 2019 Rover Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# CMake script for finding Magick++, the C++ interface for the
# GraphicsMagick library
#
# Output variables:
#  GRAPHICSMAGICKCPP_FOUND        - system has GraphicsMagick Magick++
#  GRAPHICSMAGICKCPP_INCLUDE_DIRS - include directories for Magick++
#  GRAPHICSMAGICKCPP_LIBRARIES    - libraries you need to link to
include(FindPackageHandleStandardArgs)

find_path(GRAPHICSMAGICKCPP_INCLUDE_DIRS
  NAMES "Magick++.h"
  PATH_SUFFIXES GraphicsMagick)

find_library(GRAPHICSMAGICKCPP_LIBRARIES
  NAMES "GraphicsMagick++" "graphicsmagick")

find_package_handle_standard_args(
  GRAPHICSMAGICKCPP
  GRAPHICSMAGICKCPP_LIBRARIES
  GRAPHICSMAGICKCPP_INCLUDE_DIRS)