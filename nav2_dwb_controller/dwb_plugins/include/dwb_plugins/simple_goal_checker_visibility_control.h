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

#ifndef DWB_PLUGINS_SIMPLE_GOAL_CHECKER__VISIBILITY_CONTROL_H_
#define DWB_PLUGINS_SIMPLE_GOAL_CHECKER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_EXPORT __attribute__ ((dllexport))
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_IMPORT __attribute__ ((dllimport))
  #else
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_EXPORT __declspec(dllexport)
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DWB_PLUGINS_SIMPLE_GOAL_CHECKER_BUILDING_DLL
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_PUBLIC DWB_PLUGINS_SIMPLE_GOAL_CHECKER_EXPORT
  #else
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_PUBLIC DWB_PLUGINS_SIMPLE_GOAL_CHECKER_IMPORT
  #endif
  #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_PUBLIC_TYPE DWB_PLUGINS_SIMPLE_GOAL_CHECKER_PUBLIC
  #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_LOCAL
#else
  #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_EXPORT __attribute__ ((visibility("default")))
  #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_IMPORT
  #if __GNUC__ >= 4
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_PUBLIC __attribute__ ((visibility("default")))
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_PUBLIC
    #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_LOCAL
  #endif
  #define DWB_PLUGINS_SIMPLE_GOAL_CHECKER_PUBLIC_TYPE
#endif

#endif  // DWB_PLUGINS_SIMPLE_GOAL_CHECKER__VISIBILITY_CONTROL_H_
