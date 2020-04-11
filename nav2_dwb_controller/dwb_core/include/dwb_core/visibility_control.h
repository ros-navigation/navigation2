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

#ifndef DWB_CORE__VISIBILITY_CONTROL_H_
#define DWB_CORE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DWB_CORE_EXPORT __attribute__ ((dllexport))
    #define DWB_CORE_IMPORT __attribute__ ((dllimport))
  #else
    #define DWB_CORE_EXPORT __declspec(dllexport)
    #define DWB_CORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DWB_CORE_BUILDING_DLL
    #define DWB_CORE_PUBLIC DWB_CORE_EXPORT
  #else
    #define DWB_CORE_PUBLIC DWB_CORE_IMPORT
  #endif
  #define DWB_CORE_PUBLIC_TYPE DWB_CORE_PUBLIC
  #define DWB_CORE_LOCAL
#else
  #define DWB_CORE_EXPORT __attribute__ ((visibility("default")))
  #define DWB_CORE_IMPORT
  #if __GNUC__ >= 4
    #define DWB_CORE_PUBLIC __attribute__ ((visibility("default")))
    #define DWB_CORE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DWB_CORE_PUBLIC
    #define DWB_CORE_LOCAL
  #endif
  #define DWB_CORE_PUBLIC_TYPE
#endif

#endif  // DWB_CORE__VISIBILITY_CONTROL_H_
