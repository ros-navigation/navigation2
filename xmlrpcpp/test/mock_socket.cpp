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

#define xmlrpcpp_EXPORTS  // we are mocking XmlRpcSocket, define the symbol in order to export XmlRpcSocket class
# include "xmlrpcpp/XmlRpcSocket.h"
#undef xmlrpcpp_EXPORTS

#include "xmlrpcpp/XmlRpcUtil.h"
#include "mock_socket.h"

#include <deque>

#include <string.h>
#include <errno.h>

#include <gtest/gtest.h>

using namespace XmlRpc;

bool XmlRpcSocket::s_use_ipv6_ = false;

// Returns message corresponding to last errno.
// NOTE(austin): this matches the default implementation.
std::string XmlRpcSocket::getErrorMsg() {
  return getErrorMsg(getError());
}

// Returns message corresponding to errno
// NOTE(austin): this matches the default implementation.
std::string XmlRpcSocket::getErrorMsg(int error) {
  char err[60];
#ifdef _MSC_VER
  strerror_s(err, 60, error);
#else
  snprintf(err, sizeof(err), "%s", strerror(error));
#endif
  return std::string(err);
}

#define EXPECT_PROLOGUE(name)                                                  \
  EXPECT_EQ(0, name##_calls)                                                   \
      << "Test error; cannont expect " #name " more than once";

std::deque<int> close_calls;
void XmlRpcSocket::close(int fd) {
  EXPECT_LE(1u, close_calls.size());
  if (close_calls.size() > 0) {
    int close_fd = close_calls.front();
    close_calls.pop_front();
    EXPECT_EQ(close_fd, fd);
  }
}

void MockSocketTest::Expect_close(int fd) {
  close_calls.push_back(fd);
}

int socket_ret = 0;
int socket_calls = 0;
int XmlRpcSocket::socket() {
  EXPECT_EQ(1, socket_calls);
  socket_calls--;
  return socket_ret;
}

void MockSocketTest::Expect_socket(int ret) {
  EXPECT_PROLOGUE(socket);
  socket_calls = 1;
  socket_ret = ret;
}

bool setNonBlocking_ret = true;
int setNonBlocking_fd = 0;
int setNonBlocking_calls = 0;
bool XmlRpcSocket::setNonBlocking(int fd) {
  EXPECT_EQ(1, setNonBlocking_calls);
  setNonBlocking_calls--;
  EXPECT_EQ(setNonBlocking_fd, fd);
  return setNonBlocking_ret;
}
void MockSocketTest::Expect_setNonBlocking(int fd, bool ret) {
  EXPECT_PROLOGUE(setNonBlocking);
  setNonBlocking_calls = 1;
  setNonBlocking_fd = fd;
  setNonBlocking_ret = ret;
}

bool setReuseAddr_ret = true;
int setReuseAddr_fd = 0;
int setReuseAddr_calls = 0;
bool XmlRpcSocket::setReuseAddr(int fd) {
  EXPECT_EQ(1, setReuseAddr_calls);
  setReuseAddr_calls--;
  EXPECT_EQ(setReuseAddr_fd, fd);
  return setReuseAddr_ret;
}

void MockSocketTest::Expect_setReuseAddr(int fd, bool ret) {
  EXPECT_PROLOGUE(setReuseAddr);
  setReuseAddr_calls = 1;
  setReuseAddr_fd = fd;
  setReuseAddr_ret = ret;
}

bool bind_ret = true;
int bind_fd = 0;
int bind_port = 0;
int bind_calls = 0;
bool XmlRpcSocket::bind(int fd, int port) {
  bind_calls--;
  EXPECT_EQ(bind_fd, fd);
  EXPECT_EQ(bind_port, port);
  return bind_ret;
}

void MockSocketTest::Expect_bind(int fd, int port, bool ret) {
  EXPECT_PROLOGUE(bind);
  bind_calls = 1;
  bind_fd = fd;
  bind_port = port;
  bind_ret = ret;
}

bool listen_ret = true;
int listen_fd = 0;
int listen_backlog = 0;
int listen_calls = 0;
bool XmlRpcSocket::listen(int fd, int backlog) {
  listen_calls--;
  EXPECT_EQ(listen_fd, fd);
  EXPECT_EQ(listen_backlog, backlog);
  return listen_ret;
}

void MockSocketTest::Expect_listen(int fd, int backlog, bool ret) {
  EXPECT_PROLOGUE(listen);
  listen_calls = 1;
  listen_fd = fd;
  listen_backlog = backlog;
  listen_ret = ret;
}

int accept_ret = 0;
int accept_fd = 0;
int accept_calls = 0;
int XmlRpcSocket::accept(int fd) {
  accept_calls--;
  EXPECT_EQ(accept_fd, fd);
  return accept_ret;
}

void MockSocketTest::Expect_accept(int fd, int ret) {
  EXPECT_PROLOGUE(accept);
  accept_calls = 1;
  accept_fd = fd;
  accept_ret = ret;
}

bool connect_ret = true;
int connect_fd = 0;
std::string connect_host = "";
int connect_port = 0;
int connect_calls = 0;
bool XmlRpcSocket::connect(int fd, const std::string& host, int port) {
  connect_calls--;
  EXPECT_EQ(connect_fd, fd);
  EXPECT_EQ(connect_host, host);
  EXPECT_EQ(connect_port, port);
  return connect_ret;
}

void MockSocketTest::Expect_connect(int fd,
                                    const std::string& host,
                                    int port,
                                    bool ret) {
  EXPECT_PROLOGUE(connect);
  connect_calls = 1;
  connect_fd = fd;
  connect_host = host;
  connect_port = port;
  connect_ret = ret;
}

bool nbRead_ret = true;
int nbRead_fd = 0;
std::string nbRead_s = "";
bool nbRead_eof = false;
int nbRead_calls = 0;
bool XmlRpcSocket::nbRead(int fd, std::string& s, bool* eof) {
  nbRead_calls--;
  EXPECT_EQ(nbRead_fd, fd);
  s = nbRead_s;
  *eof = nbRead_eof;
  return nbRead_ret;
}

void MockSocketTest::Expect_nbRead(int fd,
                                   const std::string& s,
                                   bool eof,
                                   bool ret) {
  EXPECT_PROLOGUE(nbRead);
  nbRead_calls = 1;
  nbRead_fd = fd;
  nbRead_s = s;
  nbRead_eof = eof;
  nbRead_ret = ret;
}

bool nbWrite_ret = true;
int nbWrite_fd = 0;
std::string nbWrite_s = "";
int nbWrite_bytes = 0;
int nbWrite_calls = 0;
bool XmlRpcSocket::nbWrite(int fd, const std::string& s, int* bytesSoFar) {
  nbWrite_calls--;
  EXPECT_EQ(nbWrite_fd, fd);
  EXPECT_EQ(nbWrite_s, s);
  *bytesSoFar = nbWrite_bytes;
  return nbWrite_ret;
}

void MockSocketTest::Expect_nbWrite(int fd,
                                    const std::string& s,
                                    int bytes,
                                    bool ret) {
  EXPECT_PROLOGUE(nbWrite);
  nbWrite_calls = 1;
  nbWrite_fd = fd;
  nbWrite_s = s;
  nbWrite_bytes = bytes;
  nbWrite_ret = ret;
}

int getError_ret = 0;
int getError_calls = 0;
int XmlRpcSocket::getError() {
  getError_calls--;
  return getError_ret;
}

void MockSocketTest::Expect_getError(int ret) {
  EXPECT_PROLOGUE(getError);
  getError_calls = 1;
  getError_ret = ret;
}

int get_port_ret = 0;
int get_port_socket = 0;
int get_port_calls = 0;
int XmlRpcSocket::get_port(int socket) {
  get_port_calls--;
  EXPECT_EQ(get_port_socket, socket);
  return get_port_ret;
}

void MockSocketTest::Expect_get_port(int socket, int ret) {
  EXPECT_PROLOGUE(get_port);
  get_port_calls = 1;
  get_port_socket = socket;
  get_port_ret = ret;
}

void MockSocketTest::SetUp() {
  socket_calls = 0;
  close_calls.clear();
  setNonBlocking_calls = 0;
  setReuseAddr_calls = 0;
  bind_calls = 0;
  listen_calls = 0;
  accept_calls = 0;
  connect_calls = 0;
  nbRead_calls = 0;
  nbWrite_calls = 0;
  getError_calls = 0;
  get_port_calls = 0;
}

void MockSocketTest::TearDown() {
  CheckCalls();
}

void MockSocketTest::CheckCalls() {
  // Check that call counters and queues are empty.
  EXPECT_EQ(0, socket_calls);
  EXPECT_EQ(0u, close_calls.size());
  EXPECT_EQ(0, setNonBlocking_calls);
  EXPECT_EQ(0, setReuseAddr_calls);
  EXPECT_EQ(0, bind_calls);
  EXPECT_EQ(0, listen_calls);
  EXPECT_EQ(0, accept_calls);
  EXPECT_EQ(0, connect_calls);
  EXPECT_EQ(0, nbRead_calls);
  EXPECT_EQ(0, nbWrite_calls);
  EXPECT_EQ(0, getError_calls);
  EXPECT_EQ(0, get_port_calls);

  // Reset call counters and queues so we don't get leakage between different
  // parts of the test.
  socket_calls = 0;
  close_calls.clear();
  setNonBlocking_calls = 0;
  setReuseAddr_calls = 0;
  bind_calls = 0;
  listen_calls = 0;
  accept_calls = 0;
  connect_calls = 0;
  nbRead_calls = 0;
  nbWrite_calls = 0;
  getError_calls = 0;
  get_port_calls = 0;
}
