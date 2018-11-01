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

#include "xmlrpcpp/XmlRpcUtil.h"
#include "xmlrpcpp/XmlRpcSocket.h"
#include "test_system_mocks.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <netdb.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <deque>

#include <gtest/gtest.h>

#define FOR_ERRNO(itr, var, ...)                                               \
  int var[] = {__VA_ARGS__};                                                   \
  for (size_t itr = 0; itr < sizeof(var) / sizeof(int); itr++)

using XmlRpc::XmlRpcSocket;

class XmlRpcSocketTest : public ::testing::Test {
protected:
  void SetUp() {
    accept_calls = 0;
    bind_calls = 0;
    close_calls = 0;
    connect_calls = 0;
    fcntl_calls = 0;
    listen_calls = 0;
    read_calls = 0;
    select_calls = 0;
    socket_calls = 0;
    write_calls = 0;

    XmlRpc::setVerbosity(5);
    XmlRpcSocket::s_use_ipv6_ = false;
  }

  void TearDown() {
    fake_accept = 0;
    fake_bind = 0;
    fake_close = 0;
    fake_connect = 0;
    fake_fcntl = 0;
    fake_listen = 0;
    fake_read = 0;
    fake_select = 0;
    fake_socket = 0;
    fake_write = 0;
  }
};

TEST_F(XmlRpcSocketTest, TestMocks) {
  EXPECT_EQ(0, fake_accept);
  EXPECT_EQ(0, accept_calls);
  fake_accept = count_accept;
  EXPECT_EQ(0, accept(0, 0, 0));
  EXPECT_EQ(1, accept_calls);

  EXPECT_EQ(0, fake_bind);
  EXPECT_EQ(0, bind_calls);
  fake_bind = count_bind;
  EXPECT_EQ(0, bind(0, 0, 0));
  EXPECT_EQ(1, bind_calls);

  EXPECT_EQ(0, fake_close);
  EXPECT_EQ(0, close_calls);
  fake_close = count_close;
  EXPECT_EQ(0, close(-1));
  EXPECT_EQ(1, close_calls);

  EXPECT_EQ(0, fake_connect);
  EXPECT_EQ(0, connect_calls);
  fake_connect = count_connect;
  EXPECT_EQ(0, connect(0, 0, 0));
  EXPECT_EQ(1, connect_calls);

  EXPECT_EQ(0, fake_fcntl);
  EXPECT_EQ(0, fcntl_calls);
  fake_fcntl = count_fcntl;
  EXPECT_EQ(0, fcntl(0, 0, 0));
  EXPECT_EQ(1, fcntl_calls);

  EXPECT_EQ(0, fake_freeaddrinfo);
  EXPECT_EQ(0, freeaddrinfo_calls);
  fake_freeaddrinfo = count_freeaddrinfo;
  freeaddrinfo(0);
  EXPECT_EQ(1, freeaddrinfo_calls);

  EXPECT_EQ(0, fake_getaddrinfo);
  EXPECT_EQ(0, getaddrinfo_calls);
  fake_getaddrinfo = count_getaddrinfo;
  EXPECT_EQ(0, getaddrinfo(0, 0, 0, 0));
  EXPECT_EQ(1, getaddrinfo_calls);

  EXPECT_EQ(0, fake_getsockname);
  EXPECT_EQ(0, getsockname_calls);
  fake_getsockname = count_getsockname;
  EXPECT_EQ(0, getsockname(0, 0, 0));
  EXPECT_EQ(1, getsockname_calls);

  EXPECT_EQ(0, fake_listen);
  EXPECT_EQ(0, listen_calls);
  fake_listen = count_listen;
  EXPECT_EQ(0, listen(0, 0));
  EXPECT_EQ(1, listen_calls);

  EXPECT_EQ(0, fake_read);
  EXPECT_EQ(0, read_calls);
  fake_read = count_read;
  EXPECT_EQ(0, read(0, 0, 0));
  EXPECT_EQ(1, read_calls);

  EXPECT_EQ(0, fake_setsockopt);
  EXPECT_EQ(0, setsockopt_calls);
  fake_setsockopt = count_setsockopt;
  EXPECT_EQ(0, setsockopt(0, 0, 0, 0, 0));
  EXPECT_EQ(1, setsockopt_calls);

  EXPECT_EQ(0, fake_select);
  EXPECT_EQ(0, select_calls);
  fake_select = count_select;
  EXPECT_EQ(0, select(0, 0, 0, 0, 0));
  EXPECT_EQ(1, select_calls);

  EXPECT_EQ(0, fake_socket);
  EXPECT_EQ(0, socket_calls);
  fake_socket = count_socket;
  EXPECT_EQ(0, socket(0, 0, 0));
  EXPECT_EQ(1, socket_calls);

  EXPECT_EQ(0, fake_write);
  EXPECT_EQ(0, write_calls);
  fake_write = count_write;
  EXPECT_EQ(0, write(0, 0, 0));
  EXPECT_EQ(1, write_calls);
}

int socket_ret = 0;
int socket_errno = 0;
int socket_domain = 0;
int socket_type = 0;
int socket_protocol = 0;
int test_socket(int domain, int type, int protocol) {
  socket_domain = domain;
  socket_type = type;
  socket_protocol = protocol;

  socket_calls++;
  errno = socket_errno;
  return socket_ret;
}

TEST_F(XmlRpcSocketTest, socket) {
  fake_socket = test_socket;

  errno = 0;
  EXPECT_EQ(0, errno);

  socket_ret = 7;
  socket_errno = 0;
  socket_calls = 0;
  EXPECT_EQ(7, XmlRpcSocket::socket());
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(1, socket_calls);
  EXPECT_EQ(AF_INET, socket_domain);
  EXPECT_EQ(SOCK_STREAM, socket_type);
  EXPECT_EQ(0, socket_protocol);

  // Check all of the errno values that the man page says socket can set if
  // it fails
  FOR_ERRNO(i,
            errnos,
            EACCES,
            EAFNOSUPPORT,
            EINVAL,
            EMFILE,
            ENFILE,
            ENOBUFS,
            ENOMEM,
            EPROTONOSUPPORT) {
    socket_ret = -1;
    socket_errno = errnos[i];
    socket_calls = 0;
    EXPECT_EQ(-1, XmlRpcSocket::socket());
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
    EXPECT_EQ(1, socket_calls);
    EXPECT_EQ(AF_INET, socket_domain);
    EXPECT_EQ(SOCK_STREAM, socket_type);
    EXPECT_EQ(0, socket_protocol);
  }
}

int close_fd = 0;
int close_errno = 0;
int close_ret = 0;
int test_close(int fd) {
  EXPECT_EQ(close_fd, fd);

  close_calls++;
  errno = close_errno;
  return close_ret;
}

TEST_F(XmlRpcSocketTest, close) {
  // TODO(austin): XmlRpcSocket does not check or return the return value from
  //               close
  close_fd = 8;
  close_errno = 0;
  close_ret = 0;
  close_calls = 0;
  fake_close = test_close;
  XmlRpcSocket::close(8);
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(1, close_calls);

  // TODO(austin): Close should automatically retry on EINTR but does not.
  FOR_ERRNO(i, errnos, EBADF, EINTR, EIO) {
    close_errno = errnos[i];
    close_ret = -1;
    close_calls = 0;
    XmlRpcSocket::close(8);
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
    EXPECT_EQ(1, close_calls);
  }
}

int fcntl_fd = 0;
int fcntl_cmd = 0;
unsigned long fcntl_arg = 0;
int fcntl_errno = 0;
int fcntl_ret = 0;
int test_fcntl(int fd, int cmd, unsigned long arg) {
  EXPECT_EQ(fcntl_fd, fd);
  EXPECT_EQ(fcntl_cmd, cmd);
  EXPECT_EQ(fcntl_arg, arg);

  errno = fcntl_errno;
  fcntl_calls++;
  return fcntl_ret;
}

TEST_F(XmlRpcSocketTest, setNonBlocking) {
  fake_fcntl = test_fcntl;

  fcntl_fd = 9;
  fcntl_cmd = F_SETFL;
  fcntl_arg = O_NONBLOCK;

  fcntl_calls = 0;
  fcntl_errno = 0;
  fcntl_ret = 0;
  EXPECT_TRUE(XmlRpcSocket::setNonBlocking(9));
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(1, fcntl_calls);

  // Tests for the errno values that the man page indicates might reasonably be
  // returned by F_SETFL
  FOR_ERRNO(i, errnos, EACCES, EAGAIN, EBADF) {
    fcntl_calls = 0;
    fcntl_errno = errnos[i];
    fcntl_ret = -1;
    EXPECT_FALSE(XmlRpcSocket::setNonBlocking(9));
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
    EXPECT_EQ(1, fcntl_calls);
  }
}

struct expected_read {
  expected_read(int fd, const void* buf, size_t sz)
    : fd(fd), count(4095), buf(buf), sz(sz), ret(sz), _errno(0) {}

  expected_read(int fd, int _errno)
    : fd(fd), count(4095), buf(0), sz(0), ret(-1), _errno(_errno) {}

  int fd;
  size_t count;
  const void* buf;
  size_t sz;
  ssize_t ret;
  int _errno;
};

std::deque<expected_read> expected_reads;

ssize_t mock_read(int fd, void* buf, size_t count) {
  read_calls++;

  // Check that we have another call in the queue. If not, fail the test and
  // return 0 (EOF).
  EXPECT_LE(1u, expected_reads.size());
  if (expected_reads.size() < 1) {
    errno = 0;
    return 0;
  }

  // Get the next call off the queue.
  expected_read r = expected_reads.front();
  expected_reads.pop_front();

  // Check file descriptor and count.
  EXPECT_EQ(r.fd, fd);
  EXPECT_EQ(r.count, count);

  // Sanity check on count. Man pages say a count above SSIZE_MAX is undefined,
  // so check that the count that is passed doesn't trigger undefined behavior.
  EXPECT_GT(static_cast<size_t>(SSIZE_MAX), count);

  // Check that the buffer size is less or equal to the requested buffer size.
  EXPECT_LE(r.sz, count);
  size_t cnt = std::min(count, r.sz);
  // If we have a nonzero number of bytes to copy, copy them into the output
  // buffer.
  if (cnt > 0) {
    memcpy(buf, r.buf, cnt);
  }

  // Check that the return value is what we expect it to be.
  if (r.ret >= 0) {
    EXPECT_EQ(cnt, static_cast<size_t>(r.ret));
  }

  // Update errno and return.
  errno = r._errno;
  return r.ret;
}

TEST_F(XmlRpcSocketTest, nbRead) {
  fake_read = mock_read;

  const char data[] = "read1 read2 read3 read4 read5 read6 read7 read8";
  bool eof = false;
  std::string data_out;

  // Test: read some incoming data and then return EOF.
  // This is a nominal case when reading from a blocking descriptor.
  expected_reads.push_back(expected_read(7, data, 9));
  expected_reads.push_back(expected_read(7, 0, 0));

  EXPECT_TRUE(XmlRpcSocket::nbRead(7, data_out, &eof));
  EXPECT_EQ("read1 rea", data_out);
  EXPECT_TRUE(eof);
  EXPECT_EQ(2, read_calls);
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(0u, expected_reads.size());
  expected_reads.clear();
}

#define TEST_READ(RES, ERR)                                                    \
  TEST_F(XmlRpcSocketTest, nbRead_##ERR) {                                     \
    fake_read = mock_read;                                                     \
    bool eof = false;                                                          \
    std::string data_out;                                                      \
                                                                               \
    expected_reads.push_back(expected_read(7, ERR));                           \
                                                                               \
    EXPECT_##RES(XmlRpcSocket::nbRead(7, data_out, &eof));                     \
    EXPECT_EQ("", data_out);                                                   \
    EXPECT_FALSE(eof);                                                         \
    EXPECT_EQ(1, read_calls);                                                  \
    EXPECT_EQ(ERR, XmlRpcSocket::getError());                                  \
    EXPECT_EQ(0u, expected_reads.size());                                       \
    expected_reads.clear();                                                    \
  }

// EAGAIN: fd is ok, read should be expected to return 0 bytes.
TEST_READ(TRUE, EAGAIN);

// EWOULDBLOCK: same as EAGAIN.
TEST_READ(TRUE, EWOULDBLOCK);

// EINTR: interrupted by system call. Expected behavior is that the caller
// should retry the read again immediately.
TEST_F(XmlRpcSocketTest, nbRead_EINTR) {
  fake_read = mock_read;

  bool eof = false;
  std::string data_out;

  expected_reads.push_back(expected_read(7, EINTR));
  // TODO(austin): expecting a second read causes the test to fail.
  // expected_reads.push_back(expected_read(7, 0, 0));

  EXPECT_TRUE(XmlRpcSocket::nbRead(7, data_out, &eof));
  EXPECT_EQ("", data_out);
  EXPECT_FALSE(eof);
  // TODO(austin): expecting a second read causes the test to fail.
  // EXPECT_EQ(2, read_calls);
  EXPECT_EQ(1, read_calls);
  EXPECT_EQ(EINTR, XmlRpcSocket::getError());
}

// EBADF: file descriptor is bad; read should fail.
TEST_READ(FALSE, EBADF);

// EFAULT: buf is bad; read should fail.
// TODO(austin): this failure indicates that file descriptor is good, so if
// we see this error elsewhere it indicates that we shouldn't close the socket.
// Since XmlRpcSocket is handling the buffers, maybe this should be converted
// into an assertion failure inside XmlRpcSocket, and this test should
// EXPECT_DEATH?
TEST_READ(FALSE, EFAULT);

// EINVAL: File descriptor is not for reading. read should fail.
TEST_READ(FALSE, EINVAL);

// EIO: I/O error. read should probably fail.
TEST_READ(FALSE, EIO);

// EISDIR: File descriptor is a directory. read should fail.
TEST_READ(FALSE, EISDIR);

// More errors from recv (these should also apply to read on a socket).
// EACCES: Permission denied. read should fail.
TEST_READ(FALSE, EACCES);

// ECONNREFUSED: Connection refused. read should fail.
TEST_READ(FALSE, ECONNREFUSED);

// ENOMEM: Could not allocate memory. read should fail.
TEST_READ(FALSE, ENOMEM);

// ENOTCONN: Socket is not connected. read should fail.
TEST_READ(FALSE, ENOTCONN);

struct expected_write {
  expected_write(int fd, std::string data, size_t count, size_t max_write)
    : fd(fd),
      data(data),
      count(count),
      max_write(max_write),
      ret(std::min(count, max_write)),
      _errno(0) {}

  expected_write(int fd, size_t count, ssize_t ret, int _errno)
    : fd(fd), data(""), count(count), max_write(0), ret(ret), _errno(_errno) {}

  int fd;
  std::string data;
  size_t count;
  size_t max_write;
  ssize_t ret;
  int _errno;
};

std::deque<expected_write> expected_writes;

ssize_t mock_write(int fd, const void* buf, size_t count) {
  write_calls++;

  // Check that we have another call in the queue. If not, fail the test and
  // return 0 (EOF).
  EXPECT_LE(1u, expected_writes.size());
  if (expected_writes.size() < 1) {
    // Since the socket is supposed to be non-blocking, return EWOULDBLOCK
    // if we can't write.
    errno = EWOULDBLOCK;
    return -1;
  }

  expected_write w = expected_writes.front();
  expected_writes.pop_front();

  EXPECT_EQ(w.fd, fd);
  EXPECT_EQ(w.count, count);
  size_t sz = std::min(w.max_write, count);
  if (sz > 0) {
    std::string data((const char*)buf, sz);
    EXPECT_EQ(w.data, data);
    EXPECT_GE(w.ret, 0u);
    EXPECT_EQ(static_cast<size_t>(w.ret), sz);
  }

  errno = w._errno;
  return w.ret;
}

TEST_F(XmlRpcSocketTest, nbWrite) {
  fake_write = mock_write;
  int count = 0;
  std::string hello = "hello world";

  // Single write for all the data.
  expected_writes.push_back(expected_write(10, "hello world", 11, 11));
  count = 0;
  write_calls = 0;
  errno = 0;
  EXPECT_TRUE(XmlRpcSocket::nbWrite(10, hello, &count));
  EXPECT_EQ(count, 11);
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(0u, expected_writes.size());
  EXPECT_EQ(1, write_calls);

  // Write in two parts, both succeed.
  expected_writes.push_back(expected_write(10, "hello", 11, 5));
  expected_writes.push_back(expected_write(10, " world", 6, 10));
  count = 0;
  write_calls = 0;
  errno = 0;
  EXPECT_TRUE(XmlRpcSocket::nbWrite(10, hello, &count));
  EXPECT_EQ(count, 11);
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(0u, expected_writes.size());
  EXPECT_EQ(2, write_calls);

  // Partial write.
  count = 0;
  write_calls = 0;
  errno = 0;
  expected_writes.push_back(expected_write(10, "hello", 11, 5));
  expected_writes.push_back(expected_write(10, 6, -1, EWOULDBLOCK));
  EXPECT_TRUE(XmlRpcSocket::nbWrite(10, hello, &count));
  EXPECT_EQ(count, 5);
  EXPECT_EQ(EWOULDBLOCK, XmlRpcSocket::getError());
  EXPECT_EQ(0u, expected_writes.size());
  EXPECT_EQ(2, write_calls);
}

#define TEST_WRITE(RES, ERR)                                                   \
  TEST_F(XmlRpcSocketTest, nbWrite_##ERR) {                                    \
    fake_write = mock_write;                                                   \
    int count = 0;                                                             \
    std::string hello = "hello world";                                         \
    errno = 0;                                                                 \
    expected_writes.push_back(expected_write(10, 11, -1, ERR));                \
    EXPECT_##RES(XmlRpcSocket::nbWrite(10, hello, &count));                    \
    EXPECT_EQ(count, 0);                                                       \
    EXPECT_EQ(ERR, XmlRpcSocket::getError());                                  \
    EXPECT_EQ(0u, expected_writes.size());                                     \
    EXPECT_EQ(1, write_calls);                                                 \
    expected_writes.clear();                                                   \
  }

TEST_WRITE(TRUE, EAGAIN);
TEST_WRITE(TRUE, EWOULDBLOCK);
TEST_WRITE(TRUE, EINTR); // TODO(austin): this should retry immediately.
TEST_WRITE(FALSE, EBADF);
TEST_WRITE(FALSE, EDESTADDRREQ);
TEST_WRITE(FALSE, EDQUOT);
TEST_WRITE(FALSE, EFAULT);
TEST_WRITE(FALSE, EFBIG);
TEST_WRITE(FALSE, EINVAL);
TEST_WRITE(FALSE, EIO);
TEST_WRITE(FALSE, ENOSPC);
TEST_WRITE(FALSE, EPIPE);
TEST_WRITE(FALSE, EACCES);
TEST_WRITE(FALSE, ECONNRESET);
TEST_WRITE(FALSE, EISCONN);
TEST_WRITE(FALSE, ENOBUFS);
TEST_WRITE(FALSE, ENOMEM);
TEST_WRITE(FALSE, ENOTCONN);

int setsockopt_ret = 0;
int setsockopt_errno = 0;
int setsockopt_sockfd = 0;
int test_setsockopt(
    int sockfd, int level, int optname, const void* optval, socklen_t optlen) {
  setsockopt_calls++;
  setsockopt_sockfd = sockfd;

  // These arguments are all fixed, so just test for them here.
  EXPECT_EQ(SOL_SOCKET, level);
  EXPECT_EQ(SO_REUSEADDR, optname);
  EXPECT_EQ(sizeof(int), optlen);
  if (sizeof(int) == optlen) {
    EXPECT_EQ(1, *(int*)optval);
  }

  errno = setsockopt_errno;
  return setsockopt_ret;
}

TEST_F(XmlRpcSocketTest, setReuseAddr) {
  fake_setsockopt = test_setsockopt;

  errno = 0;
  setsockopt_sockfd = 0;
  setsockopt_calls = 0;

  setsockopt_errno = 0;
  setsockopt_ret = 0;
  EXPECT_TRUE(XmlRpcSocket::setReuseAddr(11));
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(11, setsockopt_sockfd);
  EXPECT_EQ(1, setsockopt_calls);

  FOR_ERRNO(i, errnos, EBADF, EFAULT, EINVAL, ENOPROTOOPT, ENOTSOCK) {
    errno = 0;
    setsockopt_sockfd = 0;
    setsockopt_calls = 0;

    setsockopt_errno = errnos[i];
    setsockopt_ret = -1;
    EXPECT_FALSE(XmlRpcSocket::setReuseAddr(11));
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
    EXPECT_EQ(11, setsockopt_sockfd);
    EXPECT_EQ(1, setsockopt_calls);
  }
}

bool operator==(const in6_addr a, const in6_addr b) {
  // Delegate to IPv6 address comparison macro.
  return IN6_ARE_ADDR_EQUAL(&a, &b);
}

int bind_ret = 0;
int bind_errno = 0;
int bind_sockfd = 0;
int bind_family = 0;
int bind_port = 0;
int test_bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) {
  bind_calls++;
  EXPECT_EQ(bind_sockfd, sockfd);

  EXPECT_TRUE(NULL != addr);
  if (NULL != addr) {
    EXPECT_EQ(bind_family, addr->sa_family);
    if (AF_INET == addr->sa_family) {
      EXPECT_EQ(sizeof(struct sockaddr_in), addrlen);
      struct sockaddr_in* in_addr = (struct sockaddr_in*)addr;
      EXPECT_EQ(INADDR_ANY, ntohl(in_addr->sin_addr.s_addr));
      EXPECT_EQ(bind_port, ntohs(in_addr->sin_port));

    } else if (AF_INET6 == addr->sa_family) {
      EXPECT_EQ(sizeof(struct sockaddr_in6), addrlen);
      struct sockaddr_in6* in6_addr = (struct sockaddr_in6*)addr;
      EXPECT_EQ(in6addr_any, in6_addr->sin6_addr);
      EXPECT_EQ(bind_port, ntohs(in6_addr->sin6_port));
    } else {
      ADD_FAILURE() << "Unrecognized sockaddr family";
    }
  }

  errno = bind_errno;
  return bind_ret;
}

TEST_F(XmlRpcSocketTest, bind) {
  fake_bind = test_bind;

  // Nominal case: bind returns success.
  bind_sockfd = 12;
  bind_family = AF_INET;
  bind_port = 22;

  bind_calls = 0;
  bind_errno = 0;
  bind_ret = 0;
  EXPECT_TRUE(XmlRpcSocket::bind(12, 22));
  EXPECT_EQ(1, bind_calls);
  EXPECT_EQ(0, XmlRpcSocket::getError());

  // Errors that the man page indicates can happen for all sockets; this does
  // not include the errors that are specific to UNIX domain sockets.
  FOR_ERRNO(i, errnos, EACCES, EADDRINUSE, EBADF, EINVAL, ENOTSOCK) {
    bind_calls = 0;

    bind_family = AF_INET;
    bind_port = 22;

    bind_errno = errnos[i];
    bind_ret = -1;
    EXPECT_FALSE(XmlRpcSocket::bind(12, 22));
    EXPECT_EQ(1, bind_calls);
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
  }

  // Basic test for IPv6 functionality.
  XmlRpcSocket::s_use_ipv6_ = true;

  bind_calls = 0;

  bind_family = AF_INET6;
  bind_port = 22;

  bind_errno = 0;
  bind_ret = 0;
  EXPECT_TRUE(XmlRpcSocket::bind(12, 22));
  EXPECT_EQ(1, bind_calls);
  EXPECT_EQ(0, XmlRpcSocket::getError());
}

int getsockname_ret = 0;
int getsockname_errno = 0;
int getsockname_sockfd = 0;
void* getsockname_addr = 0;
socklen_t getsockname_len = 0;
int test_getsockname(int sockfd, struct sockaddr* addr, socklen_t* addrlen) {
  getsockname_calls++;

  EXPECT_EQ(getsockname_sockfd, sockfd);

  EXPECT_TRUE(NULL != addr);
  EXPECT_LE(getsockname_len, *addrlen);
  if (NULL != addr) {
    socklen_t len = std::min(*addrlen, getsockname_len);
    memcpy(addr, getsockname_addr, len);
  }
  EXPECT_TRUE(NULL != addrlen);
  if (NULL != addrlen) {
    *addrlen = getsockname_len;
  }

  errno = getsockname_errno;
  return getsockname_ret;
}

TEST_F(XmlRpcSocketTest, get_port) {
  fake_getsockname = test_getsockname;

  struct sockaddr_in inet_addr;
  inet_addr.sin_family = AF_INET;
  inet_addr.sin_port = htons(123);

  struct sockaddr_in6 inet6_addr;
  inet6_addr.sin6_family = AF_INET6;
  inet6_addr.sin6_port = htons(4224);

  getsockname_sockfd = 14;

  getsockname_errno = 0;
  getsockname_ret = 0;

  getsockname_calls = 0;
  getsockname_addr = &inet_addr;
  getsockname_len = sizeof(struct sockaddr_in);
  EXPECT_EQ(123, XmlRpcSocket::get_port(14));
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(1, getsockname_calls);

  getsockname_calls = 0;
  getsockname_addr = &inet6_addr;
  getsockname_len = sizeof(struct sockaddr_in6);
  EXPECT_EQ(4224, XmlRpcSocket::get_port(14));
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(1, getsockname_calls);

  getsockname_ret = -1;
  FOR_ERRNO(i, errnos, EBADF, EFAULT, EINVAL, ENOBUFS, ENOTSOCK) {
    getsockname_errno = errnos[i];

    // Errors, no data written to buffer.
    getsockname_calls = 0;
    getsockname_addr = NULL;
    getsockname_len = 0;
    EXPECT_EQ(0, XmlRpcSocket::get_port(14));
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
    EXPECT_EQ(1, getsockname_calls);

    // Same errors, but this time put valid data and expect that it is ignored.
    getsockname_calls = 0;
    getsockname_addr = &inet_addr;
    getsockname_len = sizeof(struct sockaddr_in);
    EXPECT_EQ(0, XmlRpcSocket::get_port(14));
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
    EXPECT_EQ(1, getsockname_calls);
  }
}

int listen_ret = 0;
int listen_errno = 0;
int listen_sockfd = 0;
int listen_backlog = 0;
int test_listen(int sockfd, int backlog) {
  EXPECT_EQ(listen_sockfd, sockfd);
  EXPECT_EQ(listen_backlog, backlog);

  errno = listen_errno;
  return listen_ret;
}

TEST_F(XmlRpcSocketTest, listen) {
  fake_listen = test_listen;

  listen_sockfd = 13;
  listen_backlog = 10;

  listen_ret = 0;
  listen_errno = 0;
  EXPECT_TRUE(XmlRpcSocket::listen(13, 10));
  EXPECT_EQ(0, XmlRpcSocket::getError());

  FOR_ERRNO(i, errnos, EADDRINUSE, EBADF, ENOTSOCK, EOPNOTSUPP) {
    listen_ret = -1;
    listen_errno = errnos[i];
    EXPECT_FALSE(XmlRpcSocket::listen(13, 10));
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
  }
}

int accept_ret = 0;
int accept_errno = 0;
int accept_sockfd = 0;
void* accept_addr = 0;
socklen_t accept_addrlen = 0;
int test_accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) {
  accept_calls++;
  EXPECT_EQ(accept_sockfd, sockfd);

  if (accept_addr) {
    EXPECT_TRUE(NULL != addr);
    EXPECT_TRUE(NULL != addrlen);
    if (NULL != addr && NULL != addrlen) {
      socklen_t len = std::min(accept_addrlen, *addrlen);
      memcpy(addr, accept_addr, len);
      *addrlen = accept_addrlen;
    }
  } else {
    EXPECT_EQ(NULL, addr);
    EXPECT_EQ(NULL, addrlen);
  }

  errno = accept_errno;
  return accept_ret;
}

TEST_F(XmlRpcSocketTest, accept) {
  fake_accept = test_accept;

  // Set up address. XmlRpcSocket::accept expects this, even if it isn't used.
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(45);
  addr.sin_addr.s_addr = 0xDEADBEEF;

  accept_addr = &addr;
  accept_addrlen = sizeof(struct sockaddr);

  accept_sockfd = 15;

  accept_calls = 0;
  accept_ret = 16;
  accept_errno = 0;
  EXPECT_EQ(16, XmlRpcSocket::accept(15));
  EXPECT_EQ(0, XmlRpcSocket::getError());
  EXPECT_EQ(1, accept_calls);

  FOR_ERRNO(i,
            errnos,
            EAGAIN,
            EWOULDBLOCK,
            EBADF,
            ECONNABORTED,
            EFAULT,
            EINTR, // TODO(austin): Should this retry immediately?
            EINVAL,
            EMFILE,
            ENFILE,
            ENOBUFS,
            ENOMEM,
            ENOTSOCK,
            EOPNOTSUPP,
            EPROTO,
            EPERM) {
    accept_calls = 0;
    accept_ret = -1;
    accept_errno = errnos[i];
    EXPECT_EQ(-1, XmlRpcSocket::accept(15));
    EXPECT_EQ(errnos[i], XmlRpcSocket::getError());
    EXPECT_EQ(1, accept_calls);
  }
}

// To test connect() we need mocks for getaddrinfo(), freeaddrinfo(), connect()
// and XmlRpc logging hooks to validate that the correct error messages are
// logged.

int getaddrinfo_ret = 0;
int getaddrinfo_errno = 0;
const char* getaddrinfo_node = 0;
const char* getaddrinfo_service = 0;
struct addrinfo getaddrinfo_hints = {.ai_flags = 0,
                                     .ai_family = 0,
                                     .ai_socktype = 0,
                                     .ai_protocol = 0,
                                     .ai_addrlen = 0,
                                     .ai_addr = 0,
                                     .ai_canonname = 0,
                                     .ai_next = 0};
struct addrinfo* getaddrinfo_res = 0;
int test_getaddrinfo(const char* node,
                     const char* service,
                     const struct addrinfo* hints,
                     struct addrinfo** res) {
  getaddrinfo_calls++;

  EXPECT_STREQ(getaddrinfo_node, node);
  EXPECT_STREQ(getaddrinfo_service, service);

  EXPECT_TRUE(NULL != hints);
  if (NULL != hints) {
    EXPECT_TRUE(memcmp(hints, &getaddrinfo_hints, sizeof(struct addrinfo)) ==
                0);
  }

  EXPECT_TRUE(NULL != res);
  if (NULL != res) {
    *res = getaddrinfo_res;
  }

  errno = getaddrinfo_errno;
  return getaddrinfo_ret;
}

struct addrinfo* freeaddrinfo_res = 0;
void test_freeaddrinfo(struct addrinfo* res) {
  // The man page does not indicate any errors that freeaddrinfo may encounter.
  freeaddrinfo_calls++;

  EXPECT_EQ(freeaddrinfo_res, res);

  return;
}

void EXPECT_SOCKADDR_EQ(const sockaddr* addr1, const sockaddr* addr2) {
  EXPECT_EQ((NULL == addr1), (NULL == addr2));
  if (NULL != addr1 && NULL != addr2) {
    EXPECT_EQ(addr1->sa_family, addr2->sa_family);
    if (addr1->sa_family == addr2->sa_family) {
      switch (addr1->sa_family) {
      case AF_INET: {
        const sockaddr_in* addr1_in = (const sockaddr_in*)addr1;
        const sockaddr_in* addr2_in = (const sockaddr_in*)addr2;
        EXPECT_EQ(addr1_in->sin_port, addr2_in->sin_port);
        EXPECT_EQ(addr1_in->sin_addr.s_addr, addr2_in->sin_addr.s_addr);
      } break;
      case AF_INET6: {
        const sockaddr_in6* addr1_in6 = (const sockaddr_in6*)addr1;
        const sockaddr_in6* addr2_in6 = (const sockaddr_in6*)addr2;
        EXPECT_EQ(addr1_in6->sin6_port, addr2_in6->sin6_port);
        EXPECT_EQ(addr1_in6->sin6_flowinfo, addr2_in6->sin6_flowinfo);
        EXPECT_EQ(addr1_in6->sin6_scope_id, addr2_in6->sin6_scope_id);
        EXPECT_TRUE(
            IN6_ARE_ADDR_EQUAL(&addr1_in6->sin6_addr, &addr2_in6->sin6_addr));
        for (int i = 0; i < 16; i++) {
          EXPECT_EQ(addr1_in6->sin6_addr.s6_addr[i],
                    addr2_in6->sin6_addr.s6_addr[i])
              << "IPv6 address mismstach at byte " << i;
        }
      } break;
      default:
        ADD_FAILURE() << "Unrecognized address type; cannot compare";
      }
    }
  }
}

int connect_ret = 0;
int connect_errno = 0;
int connect_sockfd = 0;
const struct sockaddr* connect_addr = 0;
socklen_t connect_addrlen = 0;
int test_connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) {
  connect_calls++;

  EXPECT_EQ(connect_sockfd, sockfd);

  EXPECT_TRUE(NULL != addr);
  EXPECT_EQ(connect_addrlen, addrlen);
  EXPECT_SOCKADDR_EQ(connect_addr, addr);

  errno = connect_errno;
  return connect_ret;
}

class XmlRpcConnectTest : public XmlRpcSocketTest,
                          XmlRpc::XmlRpcLogHandler,
                          XmlRpc::XmlRpcErrorHandler {
public:
  virtual void log(int level, const char* msg) {
    last_level = level;
    last_msg = msg;
    std::cout << "LOG(" << level << "):" << msg;
  }

  virtual void error(const char* msg) {
    last_error = msg;
    std::cout << "ERROR: " << msg;
  }

  int last_level;
  std::string last_msg;
  std::string last_error;

  void EXPECT_LOG(int level, const std::string& msg) {
    EXPECT_EQ(level, last_level);
    EXPECT_EQ(msg, last_msg);
  }

  void EXPECT_ERROR(const std::string& msg) {
    EXPECT_EQ(msg, last_error);
  }

protected:
  void SetUp() {
    XmlRpcSocketTest::SetUp();

    // Install mock functions.
    fake_getaddrinfo = test_getaddrinfo;
    fake_freeaddrinfo = test_freeaddrinfo;
    fake_connect = test_connect;

    XmlRpc::XmlRpcLogHandler::setLogHandler(this);
    XmlRpc::XmlRpcErrorHandler::setErrorHandler(this);

    // Set up address data structures for testing use.
    addr_ip4_22.sin_family = AF_INET;
    addr_ip4_22.sin_port = htons(22);
    addr_ip4_22.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    addr_ip4_404.sin_family = AF_INET;
    addr_ip4_404.sin_port = htons(404);
    addr_ip4_404.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    addr_ip6_44.sin6_family = AF_INET6;
    addr_ip6_44.sin6_port = htons(44);
    addr_ip6_44.sin6_flowinfo = 0;
    addr_ip6_44.sin6_addr = in6addr_loopback;
    addr_ip6_44.sin6_scope_id = 0;

    addr_ip6_404.sin6_family = AF_INET6;
    addr_ip6_404.sin6_port = htons(404);
    addr_ip6_404.sin6_flowinfo = 0;
    addr_ip6_404.sin6_addr = in6addr_loopback;
    addr_ip6_404.sin6_scope_id = 0;

    info_ip4.ai_flags = 0;
    info_ip4.ai_family = AF_INET;
    info_ip4.ai_socktype = SOCK_STREAM;
    info_ip4.ai_protocol = 0;
    info_ip4.ai_addrlen = sizeof(struct sockaddr_in);
    info_ip4.ai_addr = (struct sockaddr*)&addr_ip4_22;
    info_ip4.ai_canonname = 0;
    info_ip4.ai_next = 0;

    info_ip6.ai_flags = 0;
    info_ip6.ai_family = AF_INET6;
    info_ip6.ai_socktype = SOCK_STREAM;
    info_ip6.ai_protocol = 0;
    info_ip6.ai_addrlen = sizeof(struct sockaddr_in6);
    info_ip6.ai_addr = (struct sockaddr*)&addr_ip6_44;
    info_ip6.ai_canonname = 0;
    info_ip6.ai_next = 0;

    info_canon.ai_flags = 0;
    info_canon.ai_family = 0;
    info_canon.ai_socktype = 0;
    info_canon.ai_protocol = 0;
    info_canon.ai_addrlen = 0;
    info_canon.ai_addr = 0;
    info_canon.ai_canonname = 0;
    info_canon.ai_next = 0;

    // Set up expected hints.
    getaddrinfo_hints.ai_family = AF_UNSPEC;
    // TODO(austin): hints should probably specify SOCK_STREAM
    // getaddrinfo_hints.ai_socktype = SOCK_STREAM;

    getaddrinfo_node = "nowhere.com";
    getaddrinfo_service = 0;

    connect_sockfd = 19;
  }

  void TestConnect() {
    // Always free the same getaddrinfo that we started with.
    freeaddrinfo_res = getaddrinfo_res;

    // Clear last log message.
    last_level = 0;
    last_msg = "";
    last_error = "";

    // Actually start running tests.
    getaddrinfo_calls = 0;
    freeaddrinfo_calls = 0;
    connect_calls = 0;
    EXPECT_TRUE(XmlRpcSocket::connect(19, "nowhere.com", 404));
    EXPECT_EQ(1, getaddrinfo_calls);
    EXPECT_EQ(1, freeaddrinfo_calls);
    EXPECT_EQ(1, connect_calls);
  }

  void TestLookupFail() {
    // Always free the same getaddrinfo that we started with.
    freeaddrinfo_res = getaddrinfo_res;

    // Clear last log message.
    last_level = 0;
    last_msg = "";
    last_error = "";

    // Actually start running tests.
    getaddrinfo_calls = 0;
    freeaddrinfo_calls = 0;
    connect_calls = 0;
    EXPECT_FALSE(XmlRpcSocket::connect(19, "nowhere.com", 404));
    EXPECT_EQ(1, getaddrinfo_calls);
    EXPECT_EQ(0, freeaddrinfo_calls);
    EXPECT_EQ(0, connect_calls);
  }

  void TestConnectFail() {
    // Always free the same getaddrinfo that we started with.
    freeaddrinfo_res = getaddrinfo_res;

    // Clear last log message.
    last_level = 0;
    last_msg = "";
    last_error = "";

    // Actually start running tests.
    getaddrinfo_calls = 0;
    freeaddrinfo_calls = 0;
    connect_calls = 0;
    EXPECT_FALSE(XmlRpcSocket::connect(19, "nowhere.com", 404));
    EXPECT_EQ(1, getaddrinfo_calls);
    EXPECT_EQ(1, freeaddrinfo_calls);
    EXPECT_EQ(1, connect_calls);
  }

  void TearDown() {
    XmlRpcLogHandler::setLogHandler(NULL);
    XmlRpcErrorHandler::setErrorHandler(NULL);

    XmlRpcSocketTest::TearDown();
  }

  struct sockaddr_in addr_ip4_22;
  struct sockaddr_in addr_ip4_404;
  struct sockaddr_in6 addr_ip6_44;
  struct sockaddr_in6 addr_ip6_404;
  struct addrinfo info_ip4;
  struct addrinfo info_ip6;
  struct addrinfo info_canon;
};

TEST_F(XmlRpcConnectTest, connect_ipv4) {
  // Expected results from getaddrinfo.
  getaddrinfo_ret = 0;
  getaddrinfo_errno = 0;

  // Expected results from connect.
  connect_ret = 0;
  connect_errno = 0;
  connect_addr = (struct sockaddr*)&addr_ip4_404;
  connect_addrlen = sizeof(struct sockaddr_in);

  // Canonical address first, then IPv4, IPv6.
  getaddrinfo_res = &info_canon;
  info_canon.ai_next = &info_ip4;
  info_ip4.ai_next = &info_ip6;
  info_ip6.ai_next = 0;
  TestConnect();

  // List with IPv6 first.
  getaddrinfo_res = &info_canon;
  info_canon.ai_next = &info_ip6;
  info_ip6.ai_next = &info_ip4;
  info_ip4.ai_next = 0;
  TestConnect();

  // List without canon address, IPv6 first.
  getaddrinfo_res = &info_ip6;
  info_ip6.ai_next = &info_ip4;
  info_ip4.ai_next = 0;
  TestConnect();

  // List without canon address, IPv4 first.
  getaddrinfo_res = &info_ip4;
  info_ip4.ai_next = &info_ip6;
  info_ip6.ai_next = 0;
  TestConnect();
}

TEST_F(XmlRpcConnectTest, connect_ipv6) {
  // Basic IPv6 tests.
  XmlRpcSocket::s_use_ipv6_ = true;

  // Expected results from getaddrinfo.
  getaddrinfo_ret = 0;
  getaddrinfo_errno = 0;

  // Expected results from connect.
  connect_ret = 0;
  connect_errno = 0;
  connect_addr = (struct sockaddr*)&addr_ip6_404;
  connect_addrlen = sizeof(struct sockaddr_in6);

  // List without canon address, IPv4 first.
  getaddrinfo_res = &info_ip4;
  info_ip4.ai_next = &info_ip6;
  info_ip6.ai_next = 0;
  TestConnect();

  // List without canon address, IPv6 first.
  getaddrinfo_res = &info_ip6;
  info_ip6.ai_next = &info_ip4;
  info_ip4.ai_next = 0;
  TestConnect();
}

// Simulate error returns from getaddrinfo. Check that the error is logged
// correctly, that connect is not called, that the result is freed(or not)
// and that connect correctly returns an error.
TEST_F(XmlRpcConnectTest, connect_lookup_fail) {
  // Expected results from connect.
  connect_ret = 0;
  connect_errno = 0;
  connect_addr = (struct sockaddr*)&addr_ip4_404;
  connect_addrlen = sizeof(struct sockaddr_in);

  // List without canon address, IPv4 first.
  getaddrinfo_res = &info_ip4;
  info_ip4.ai_next = &info_ip6;
  info_ip6.ai_next = 0;

  // Enumerate possible return codes from getaddrinfo and make sure that
  // connect() fails and that the correct error message is reported.
  FOR_ERRNO(i,
            addr_errs,
            EAI_ADDRFAMILY,
            EAI_AGAIN,
            EAI_BADFLAGS,
            EAI_FAIL,
            EAI_FAMILY,
            EAI_MEMORY,
            EAI_NODATA,
            EAI_NONAME,
            EAI_SERVICE,
            EAI_SOCKTYPE) {
    // Results from getaddrinfo.
    getaddrinfo_ret = addr_errs[i];
    getaddrinfo_errno = 0;

    TestLookupFail();
    EXPECT_ERROR(
        std::string("Couldn't find an AF_INET address for [nowhere.com]: ") +
        std::string(gai_strerror(addr_errs[i])) + std::string("\n"));
  }

  // Enumerate system failures from getaddrinfo and make sure that connect()
  // fails and that the correct error from perror is reported.
  // TODO(austin): getaddrinfo should retry on EINTR but it doesn't.
  getaddrinfo_ret = EAI_SYSTEM;
  FOR_ERRNO(i, errnos, ENFILE, EMFILE, EAGAIN, EINTR) {
    getaddrinfo_errno = errnos[i];
    TestLookupFail();
    EXPECT_ERROR(
        std::string("Couldn't find an AF_INET address for [nowhere.com]: ") +
        std::string(strerror(errnos[i])) + std::string("\n"));
  }

  // IPv4 lookup only returns IPv6 results.
  getaddrinfo_res = &info_ip6;
  info_ip6.ai_next = 0;

  // Results from getaddrinfo.
  getaddrinfo_ret = 0;
  getaddrinfo_errno = 0;

  // Always free the same getaddrinfo that we started with.
  freeaddrinfo_res = getaddrinfo_res;

  // Clear last log message.
  last_level = 0;
  last_msg = "";
  last_error = "";

  // Call connect and analyze results. We do this inline here instead of
  // using one of the convenience functions because none of the convenience
  // functions fit this pattern.
  getaddrinfo_calls = 0;
  freeaddrinfo_calls = 0;
  connect_calls = 0;
  EXPECT_FALSE(XmlRpcSocket::connect(19, "nowhere.com", 404));
  EXPECT_EQ(1, getaddrinfo_calls);
  EXPECT_EQ(1, freeaddrinfo_calls);
  EXPECT_EQ(0, connect_calls);
  EXPECT_ERROR(
      std::string("Couldn't find an AF_INET address for [nowhere.com]") +
      std::string("\n"));
}

// TODO(austin): this probably won't work, isn't supported upstream and isn't
// required by our current use case. Future work.
// Simulate multiple results for a hostname lookup. Simulate failure to
// connect to the first hostname and verify that another attempt is made
// for the second address.
TEST_F(XmlRpcConnectTest, connect_multiple_result) {}

// Simulate various error results from connect().
TEST_F(XmlRpcConnectTest, connect_failure) {
  // Expected results from getaddrinfo.
  getaddrinfo_ret = 0;
  getaddrinfo_errno = 0;

  // Expected results from connect.
  connect_ret = 0;
  connect_errno = 0;
  connect_addr = (struct sockaddr*)&addr_ip4_404;
  connect_addrlen = sizeof(struct sockaddr_in);

  // List without canon address, IPv4 first.
  getaddrinfo_res = &info_ip4;
  info_ip4.ai_next = &info_ip6;
  info_ip6.ai_next = 0;

  // EINPROGRESS indicates that the socket is non-blocking and the connection
  // request has started but not finished, so we expect success.
  connect_ret = -1;
  connect_errno = EINPROGRESS;
  TestConnect();
  EXPECT_ERROR("");

  // On Windows, EWOULDBLOCK (really WSAEWOULDBLOCK) indicates that the
  // socket is non-blocking and could not be completed immediately (success)
  // but on Linux it it synonymous with EAGAIN which indicates that there are
  // no more available local ports.
  //
  // NOTE(austin): if this comparison fails, create separate tests for EAGAIN
  //               and EWOULDBLOCK.
  EXPECT_EQ(EWOULDBLOCK, EAGAIN);
  connect_ret = -1;
  connect_errno = EWOULDBLOCK;
#if defined(_WINDOWS)
  TestConnect();
  EXPECT_ERROR("");
#else
  TestConnectFail();
  EXPECT_ERROR(std::string("::connect error = ") +
               std::string(strerror(EWOULDBLOCK)) + std::string("\n"));
#endif

  // All other errors that connect may return should be reported as an error.
  // TODO(austin): Connect should immediately retry on EINTR but it doesn't.
  FOR_ERRNO(i,
            errnos,
            EACCES,
            EPERM,
            EADDRINUSE,
            EAFNOSUPPORT,
            EALREADY,
            EBADF,
            ECONNREFUSED,
            EFAULT,
            EINTR,
            EISCONN,
            ENETUNREACH,
            ENOTSOCK,
            ETIMEDOUT) {
    connect_ret = -1;
    connect_errno = errnos[i];
    TestConnectFail();
    EXPECT_ERROR(std::string("::connect error = ") +
                 std::string(strerror(errnos[i])) + std::string("\n"));
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
