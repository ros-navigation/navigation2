// Copyright 2021 Víctor Mayoral-Vilches
// Copyright 2019 Robert Bosch GmbH
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

#include <stdio.h>
#include "nav2_tracetools/tracetools.h"

int main()
{
#ifndef TRACETOOLS_DISABLED
  printf("Tracing ");
  if (ros_trace_compile_status()) {
    printf("enabled\n");
    return 0;
  } else {
    printf("disabled\n");
    return 1;
  }
#else
  printf("Tracing disabled through configuration\n");
  return 1;
#endif
}
