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

#include "nav2_tracetools/config.h"

#ifdef TRACETOOLS_LTTNG_ENABLED
#include <dlfcn.h>
#include <cxxabi.h>
#endif
#include "nav2_tracetools/utils.hpp"

const char * _demangle_symbol(const char * mangled)
{
#ifdef TRACETOOLS_LTTNG_ENABLED
  char * demangled = nullptr;
  int status;
  demangled = abi::__cxa_demangle(mangled, NULL, 0, &status);
  // Use demangled symbol if possible
  const char * demangled_val = (status == 0 ? demangled : mangled);
  return demangled_val != 0 ? demangled_val : "UNKNOWN_demangling_failed";
#else
  (void)mangled;
  return "DISABLED__demangle_symbol";
#endif
}

const char * _get_symbol_funcptr(void * funcptr)
{
#ifdef TRACETOOLS_LTTNG_ENABLED
  Dl_info info;
  if (dladdr(funcptr, &info) == 0) {
    return SYMBOL_UNKNOWN;
  }
  return _demangle_symbol(info.dli_sname);
#else
  (void)funcptr;
  return "DISABLED__get_symbol_funcptr";
#endif
}
