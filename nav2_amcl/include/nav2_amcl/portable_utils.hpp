// Copyright (c) Samsung Research America
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
#ifndef NAV2_AMCL__PORTABLE_UTILS_HPP_
#define NAV2_AMCL__PORTABLE_UTILS_HPP_

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_DRAND48
// Some system (e.g., Windows) doesn't come with drand48(), srand48().
// Use rand, and srand for such system.
static double drand48(void)
{
  return ((double)rand()) / RAND_MAX;// NOLINT
}

static void srand48(long int seedval)// NOLINT
{
  srand(seedval);
}
#endif

#ifdef __cplusplus
}
#endif

#endif  // NAV2_AMCL__PORTABLE_UTILS_HPP_
