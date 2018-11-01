/*
 * Unit tests for XmlRpc++
 *
 * Copyright (C) 2017, Zoox Inc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Austin Hendrix <austin@zoox.com>
 *
 */

// The auto-generated mocks here don't use their parameters, so we disable
// that warning.
#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#define MOCK_SYSCALL(ret, name, ARG_TYPES, ARG_NAMES)                          \
  ret(*fake_##name) ARG_TYPES = 0;                                             \
  ret __wrap_##name ARG_TYPES {                                                \
    if (fake_##name) {                                                         \
      return fake_##name ARG_NAMES;                                            \
    } else {                                                                   \
      return -1;                                                                \
    }                                                                          \
  }                                                                            \
  int name##_calls = 0;                                                        \
  ret count_##name ARG_TYPES {                                                 \
    name##_calls++;                                                            \
    return 0;                                                                  \
  }

#include "test_system_mocks.h"

// custom mock for fcntl because it is varargs
// the mocked version always takes the third argument
int (*fake_fcntl)(int fd, int cmd, unsigned long) = 0;
int __wrap_fcntl(int fd, int cmd, ...) {
  va_list ap;
  va_start(ap, cmd);
  unsigned long arg = va_arg(ap, unsigned long);
  va_end(ap);

  if (fake_fcntl) {
    return fake_fcntl(fd, cmd, arg);
  } else {
    return -1;
  }
}
int fcntl_calls = 0;
int count_fcntl(int fd, int cmd, unsigned long arg) {
  fcntl_calls++;
  return 0;
}

// Custom mock for freeaddrinfo because it returns void.
void (*fake_freeaddrinfo)(struct addrinfo* res) = 0;
void __wrap_freeaddrinfo(struct addrinfo* res) {
  if (fake_freeaddrinfo) {
    return fake_freeaddrinfo(res);
  } else {
    return;
  }
}
int freeaddrinfo_calls = 0;
void count_freeaddrinfo(struct addrinfo* res) {
  freeaddrinfo_calls++;
  return;
}
