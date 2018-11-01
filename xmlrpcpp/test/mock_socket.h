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

#pragma once
#include <string>
#include <gtest/gtest.h>

class MockSocketTest : public ::testing::Test {
protected:
  void SetUp();

  void CheckCalls();
  void TearDown();

  void Expect_socket(int ret);
  void Expect_close(int fd);
  void Expect_setNonBlocking(int fd, bool ret);
  void Expect_setReuseAddr(int fd, bool ret);
  void Expect_bind(int fd, int port, bool ret);
  void Expect_listen(int fd, int backlog, bool ret);
  void Expect_accept(int fd, int ret);
  void Expect_connect(int fd, const std::string& host, int port, bool ret);
  void Expect_nbRead(int fd, const std::string& s, bool eof, bool ret);
  void Expect_nbWrite(int fd, const std::string& s, int bytes, bool ret);
  void Expect_getError(int ret);
  void Expect_get_port(int socket, int ret);
};
