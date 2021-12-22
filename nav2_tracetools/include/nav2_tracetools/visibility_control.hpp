// Copyright 2021 Víctor Mayoral-Vilches
// Copyright 2015 Open Source Robotics Foundation, Inc.
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

/* This header must be included by all TRACETOOLS headers which declare symbols
 * which are defined in the TRACETOOLS library. When not building the TRACETOOLS
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the TRACETOOLS
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef NAV2_TRACETOOLS__VISIBILITY_CONTROL_HPP_
#define NAV2_TRACETOOLS__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRACETOOLS_EXPORT __attribute__ ((dllexport))
    #define TRACETOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define TRACETOOLS_EXPORT __declspec(dllexport)
    #define TRACETOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRACETOOLS_BUILDING_DLL
    #define TRACETOOLS_PUBLIC TRACETOOLS_EXPORT
  #else
    #define TRACETOOLS_PUBLIC TRACETOOLS_IMPORT
  #endif
  #define TRACETOOLS_PUBLIC_TYPE TRACETOOLS_PUBLIC
  #define TRACETOOLS_LOCAL
#else
  #define TRACETOOLS_EXPORT __attribute__ ((visibility("default")))
  #define TRACETOOLS_IMPORT
  #if __GNUC__ >= 4
    #define TRACETOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define TRACETOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRACETOOLS_PUBLIC
    #define TRACETOOLS_LOCAL
  #endif
  #define TRACETOOLS_PUBLIC_TYPE
#endif

#endif  // NAV2_TRACETOOLS__VISIBILITY_CONTROL_HPP_
