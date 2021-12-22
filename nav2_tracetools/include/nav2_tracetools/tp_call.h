// Copyright 2021 Víctor Mayoral-Vilches
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

// Provide fake header guard for cpplint
#undef NAV2_TRACETOOLS__TP_CALL_H_
#ifndef NAV2_TRACETOOLS__TP_CALL_H_
#define NAV2_TRACETOOLS__TP_CALL_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ros2_navigation2

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "nav2_tracetools/tp_call.h"

#if !defined(_NAV2_TRACETOOLS__TP_CALL_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _NAV2_TRACETOOLS__TP_CALL_H_

#include <lttng/tracepoint.h>

#include <stdint.h>
#include <stdbool.h>

// Add tracepoints

#endif  // _NAV2_TRACETOOLS__TP_CALL_H_

#include <lttng/tracepoint-event.h>

#endif  // NAV2_TRACETOOLS__TP_CALL_H_
