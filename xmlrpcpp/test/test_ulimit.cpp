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

#include "test_fixtures.h"

#ifndef _WIN32
# include <sys/resource.h>
#endif

using namespace XmlRpc;

TEST_F(XmlRpcTest, Ulimit)
{
#ifndef _WIN32
  XmlRpcClient c("localhost", port);
  XmlRpcValue noArgs, result;

  // Call the Hello method
  ASSERT_TRUE(c.execute("Hello", noArgs, result));

  EXPECT_FALSE(c.isFault());
  XmlRpcValue hello("Hello");
  EXPECT_EQ(result, hello);

  c.close();
  result.clear();

  // Get the current open file limits and check that we have a reasonable
  // margin. We need to reduce the limit to 8 open files to starve the server
  // side, so we would need 9 or 10 open files for it to work correctly
  // Ensuring that we have a hard limit of at least 64 file descriptors gives
  // a very wide margin above that.
  struct rlimit limit = {.rlim_cur = 0, .rlim_max = 0};
  ASSERT_EQ(0, getrlimit(RLIMIT_NOFILE, &limit));
  ASSERT_LT(64u, limit.rlim_max);
  ASSERT_LT(64u, limit.rlim_cur);

  // Reduce the number of open file descriptors so that we can create a client
  // but can't accept the connection on the server side. 32 is more than the
  // number of currently open files, but less than minimum unused file
  // descriptors. We expect the server to be able to accept the connection and
  // then immediately reject it without servicing it.
  limit.rlim_cur = 32;
  ASSERT_EQ(0, setrlimit(RLIMIT_NOFILE, &limit));

  XmlRpcClient c2("127.0.0.1", port);
  EXPECT_FALSE(c2.execute("Hello", noArgs, result));

  // Raise the limit and verify that clients can connect again
  limit.rlim_cur = limit.rlim_max;
  ASSERT_EQ(0, setrlimit(RLIMIT_NOFILE, &limit));
  c2.close();
  EXPECT_TRUE(c2.execute("Hello", noArgs, result));
  EXPECT_EQ(result, hello);
#endif
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
