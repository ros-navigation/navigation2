// Copyright 2020 Microsoft Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV_2D_UTILS_PATH_OPS__VISIBILITY_CONTROL_H_
#define NAV_2D_UTILS_PATH_OPS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAV_2D_UTILS_PATH_OPS_EXPORT __attribute__ ((dllexport))
    #define NAV_2D_UTILS_PATH_OPS_IMPORT __attribute__ ((dllimport))
  #else
    #define NAV_2D_UTILS_PATH_OPS_EXPORT __declspec(dllexport)
    #define NAV_2D_UTILS_PATH_OPS_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAV_2D_UTILS_PATH_OPS_BUILDING_DLL
    #define NAV_2D_UTILS_PATH_OPS_PUBLIC NAV_2D_UTILS_PATH_OPS_EXPORT
  #else
    #define NAV_2D_UTILS_PATH_OPS_PUBLIC NAV_2D_UTILS_PATH_OPS_IMPORT
  #endif
  #define NAV_2D_UTILS_PATH_OPS_PUBLIC_TYPE NAV_2D_UTILS_PATH_OPS_PUBLIC
  #define NAV_2D_UTILS_PATH_OPS_LOCAL
#else
  #define NAV_2D_UTILS_PATH_OPS_EXPORT __attribute__ ((visibility("default")))
  #define NAV_2D_UTILS_PATH_OPS_IMPORT
  #if __GNUC__ >= 4
    #define NAV_2D_UTILS_PATH_OPS_PUBLIC __attribute__ ((visibility("default")))
    #define NAV_2D_UTILS_PATH_OPS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAV_2D_UTILS_PATH_OPS_PUBLIC
    #define NAV_2D_UTILS_PATH_OPS_LOCAL
  #endif
  #define NAV_2D_UTILS_PATH_OPS_PUBLIC_TYPE
#endif

#endif  // NAV_2D_UTILS_PATH_OPS__VISIBILITY_CONTROL_H_
