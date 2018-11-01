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

#include "xmlrpcpp/XmlRpcDispatch.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcSocket.h"
#include "mock_socket.h"

#include <fcntl.h>
#ifndef _WIN32
# include <netinet/in.h>
# include <poll.h>
# include <sys/socket.h>
# include <unistd.h>
#else
# include <winsock2.h> // For struct timeval
# include <ws2tcpip.h> // Must be after winsock2.h because MS didn't put proper inclusion guards in their headers.
typedef unsigned long int nfds_t;
#endif
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <errno.h>

#include <iostream>
#include <functional>

#include <gtest/gtest.h>

// Mocks for select and poll. The build file specifies -Wl,--wrap for both of
// these, so the original symbols are available as __real_xxx and any uses of
// those symbols instead use __wrap_xxx
extern "C" {
// Mock for poll
int (*fake_poll)(struct pollfd *, nfds_t, int) = 0;

int __wrap_poll(struct pollfd *fds, nfds_t nfds, int timeout) {
  if(fake_poll) {
    return fake_poll(fds, nfds, timeout);
  } else {
    return 0;
  }
}

}

int poll_calls = 0;
int poll_ret = 0;
int poll_errno = 0;
int poll_timeout = 0;
std::vector<pollfd> poll_fds;
int mock_poll(struct pollfd *fds, nfds_t nfds, int timeout) {
  EXPECT_EQ(1, poll_calls);
  poll_calls--;

  EXPECT_EQ(poll_fds.size(), nfds);
  EXPECT_EQ(poll_timeout, timeout);

  for(nfds_t i=0; i<nfds && i<poll_fds.size(); i++) {
    EXPECT_EQ(poll_fds[i].fd, fds[i].fd);
    EXPECT_EQ(poll_fds[i].events, fds[i].events);
    fds[i].revents = poll_fds[i].revents;
  }

  // HACK: Sleep for the requested duration
  // This can be thought of as a simulation of select() getting an event
  // exactly at the end of the timeout window, but really it's just here
  // because the dispatch loop has its own timer to determine if it should call
  // select again, and we don't want multiple calls.
  if(timeout > 0) {
    timespec ts;
    ts.tv_sec = timeout / 1000;
    ts.tv_nsec = (timeout % 1000) * 1000000;
    // Call nanosleep repeatedly until it returns 0 (success).
    // On failure it will update ts with the remaining time to sleep.
    int ret = 0;
    do {
#ifndef _WIN32
      ret = nanosleep(&ts, &ts);
#else
      Sleep(timeout);
#endif
    } while( ret != 0 && errno == EINTR);
  }

  errno = poll_errno;
  return poll_ret;
}

void Expect_poll(std::vector<pollfd> fds, int timeout, int _errno, int ret) {
  EXPECT_EQ(0, poll_calls) << "Test bug: Cannot expect more than one call to poll";
  poll_calls = 1;
  poll_ret = ret;
  poll_errno = _errno;
  poll_timeout = timeout;
  poll_fds = fds;
}

using namespace XmlRpc;

class MockSource : public XmlRpcSource {
public:
  MockSource(int fd)
    : handleEvent_calls(0), last_event(0), event_result(0), close_calls(0) {
    setfd(fd);
  }

  virtual ~MockSource() {
  }

  virtual unsigned handleEvent(unsigned eventType) {
    handleEvent_calls++;
    last_event = eventType;
    return event_result;
  }

  virtual void close() {
    close_calls++;
  }

  // variables used for mocking
  int handleEvent_calls;
  unsigned last_event;
  unsigned event_result;

  int close_calls;
};

#define EXPECT_CLOSE_CALLS(n)                                                  \
  do {                                                                         \
    EXPECT_EQ(m.close_calls, n);                                               \
    m.close_calls = 0;                                                         \
  } while (0)

#define EXPECT_EVENTS(n)                                                       \
  do {                                                                         \
    EXPECT_EQ(m.handleEvent_calls, n);                                         \
    m.handleEvent_calls = 0;                                                   \
  } while (0)

#define EXPECT_EVENT(event)                                                    \
  do {                                                                         \
    EXPECT_EQ(m.last_event, event);                                            \
    EXPECT_EQ(m.handleEvent_calls, 1);                                         \
    m.handleEvent_calls = 0;                                                   \
  } while (0)

class MockSourceTest : public ::testing::Test {
  protected:
    MockSourceTest() : m(4) {
#ifndef _WIN32
      pollfd f = { .fd = 4, .events = 0, .revents = 0 };
#else
      pollfd f {};
      f.fd = 4;
      f.events = 0;
      f.revents = 0;
#endif
      fds.push_back(f);
    }

    void SetUp() {
      fake_poll = mock_poll;
      poll_calls = 0;
    }

    void TearDown() {
      EXPECT_EQ(0, poll_calls);
      fake_poll = 0;
      poll_calls = 0;
    }

    MockSource m;
    XmlRpcDispatch dispatch;
    std::vector<pollfd> fds;
};

/*
 * Use a socket to provide the requisite file descriptor
 *
 * Tests to perform on XmlRpcDispatch
 * - Matrix of the following options:
 *   - Proper handling of setKeepOpen
 *   - Proper handling of deleteOnClose
 *   - Proper handling of return values from handleEvent
 * - Proper handling of file descriptor states
 *   - Correct masking of events by the eventMask for the source
 *   - Correct handling of exceptional file descriptor states
 *     - These states seem to mostly be related to sending OOB data over TCP; I
 *       don't see a way to simulate them with pipes, but it should be possible
 *       to loop back a TCP connection and generate that condition directly
 *   - Check that the argument to handleEvent matches the event that was
 *     simulated
 *   - Check that handleEvent is not called if no event was triggered
 * - Proper handling of timeout in XmlRpcDispatch work() method
 * - Proper removal of sources from _sources when errors occur
 * - If possible, trigger error return values from select(); maybe one of the
 *   following error cases from the select(2) man page:
 *    - Invalid file descriptor in set (already closed?)
 *    - Signal caught
 *    - nfds negative or invalid timeout
 *    - unable to allocate memory
 * - Proper handling of multiple XmlRpcSource objects
 *   - Multiple events during a single work() cycle
 *   - Events delivered to the correct Source
 */

TEST_F(MockSourceTest, ReadEvent) {
  m.event_result = XmlRpcDispatch::ReadableEvent;
  dispatch.addSource(&m, XmlRpcDispatch::ReadableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1u);

  // Select returns not readable; expect no events.
  fds[0].events = POLLIN;
  fds[0].revents = 0;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);

  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);

  // Select returns readable, expect readable event
  fds[0].events = POLLIN;
  fds[0].revents = POLLIN;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENT(XmlRpcDispatch::ReadableEvent);
}

TEST_F(MockSourceTest, WriteEvent) {
  m.setKeepOpen();
  m.event_result = 0;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1u);

  // Select returns writeable, expect one write event.
  fds[0].events = POLLOUT;
  fds[0].revents = POLLOUT;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENT(XmlRpcDispatch::WritableEvent);
  // We have keepOpen set, so don't expect a close call
  EXPECT_CLOSE_CALLS(0);
  // However, even if keepOpen is set, we expect the socket to be removed from
  // the sources list
  EXPECT_EQ(dispatch._sources.size(), 0u);

  // Expect no more events. Since there's nothing in the dispatch list, we
  // don't expect that select will be called.
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);
}

TEST_F(MockSourceTest, NonWriteable) {
  m.event_result = XmlRpcDispatch::WritableEvent;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1u);

  // Select doesn't return writable.
  fds[0].events = POLLOUT;
  fds[0].revents = 0;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENTS(0);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EQ(dispatch._sources.size(), 1u);
}

TEST_F(MockSourceTest, WriteClose) {
  m.event_result = 0;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1u);

  // Socket is always writeable. Expect 1 write event since we clear the write
  // event flag after we write once
  fds[0].events = POLLOUT;
  fds[0].revents = POLLOUT;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENT(XmlRpcDispatch::WritableEvent);

  // Since we returned 0 from handleEvent and don't have keepOpen set, expect
  // that the dispatch has called close() once and that the size of sources is
  // now 0
  EXPECT_CLOSE_CALLS(1);
  EXPECT_EQ(dispatch._sources.size(), 0u);

  // Expect no more events. Since there's nothing in the dispatch list, we
  // don't expect that select will be called.
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);
}

TEST_F(MockSourceTest, Exception) {
  m.event_result = XmlRpcDispatch::Exception;
  dispatch.addSource(&m, XmlRpcDispatch::Exception);
  EXPECT_EQ(dispatch._sources.size(), 1u);

  // Select returns no exception, so expect that the handler was not called.
  fds[0].events = POLLPRI;
  fds[0].revents = 0;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);

  // Make exception, expect exception event.
  fds[0].events = POLLPRI;
  fds[0].revents = POLLPRI;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENT(XmlRpcDispatch::Exception);
}

// Test that dispatch works (or doesn't) with file descriptors above 1024
TEST_F(MockSourceTest, LargeFd) {
  m.setfd(1025);
  m.event_result = XmlRpcDispatch::WritableEvent;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1u);

  // Make select return writable, expect 1 write event.
  fds[0].fd = 1025;
  fds[0].events = POLLOUT;
  fds[0].revents = POLLOUT;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENT(XmlRpcDispatch::WritableEvent);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EQ(dispatch._sources.size(), 1u);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
