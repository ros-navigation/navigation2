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

#include <gtest/gtest.h>

using namespace XmlRpc;

class FakeLogHandler : public XmlRpcLogHandler {
public:
  FakeLogHandler() : last_level(-1), last_msg(""){};

  virtual void log(int level, const char* msg) {
    last_level = level;
    last_msg = msg;
  }

  int last_level;
  std::string last_msg;
};

TEST(XmlRpc, Log) {
  FakeLogHandler fakelog;

  // Check that setting log handler is reflected in getLogHandler().
  XmlRpcLogHandler::setLogHandler(&fakelog);
  ASSERT_EQ(&fakelog, XmlRpcLogHandler::getLogHandler());

  // Check default verbosity.
  ASSERT_EQ(0, XmlRpcLogHandler::getVerbosity());
  EXPECT_EQ(0, XmlRpc::getVerbosity());

  // Test all messages masked at default verbosity.
  for (int i = 1; i < 6; i++) {
    XmlRpcUtil::log(i, "Hello");
    ASSERT_EQ(-1, fakelog.last_level);
    ASSERT_EQ("", fakelog.last_msg);
  }

  // Test masking at levels below maximum verbosity.
  for (int i = 1; i < 5; i++) {
    XmlRpc::setVerbosity(i);

    for (int j = 1; j <= i; j++) {
      XmlRpcUtil::log(j, "Hello1");
      EXPECT_EQ(j, fakelog.last_level);
      EXPECT_EQ("Hello1", fakelog.last_msg);

      fakelog.last_level = -1;
      fakelog.last_msg = "";
    }

    XmlRpcUtil::log(i + 1, "Hello2");
    ASSERT_EQ(-1, fakelog.last_level);
    ASSERT_EQ("", fakelog.last_msg);
  }

  // Test no messages masked at max verbosity.
  XmlRpc::setVerbosity(5);
  for (int i = 1; i < 5; i++) {
    XmlRpcUtil::log(i, "Hello3");
    EXPECT_EQ(i, fakelog.last_level);
    EXPECT_EQ("Hello3", fakelog.last_msg);

    fakelog.last_level = -1;
    fakelog.last_msg = "";
  }

  // Basic formatting test.
  XmlRpcUtil::log(2, "Hello %d", 42);
  EXPECT_EQ(2, fakelog.last_level);
  EXPECT_EQ("Hello 42", fakelog.last_msg);
}

class FakeErrorHandler : public XmlRpcErrorHandler {
public:
  FakeErrorHandler() : last_msg(""){};

  virtual void error(const char* msg) {
    last_msg = msg;
  }

  std::string last_msg;
};

TEST(XmlRpc, error) {
  FakeErrorHandler errors;

  // Check that setErrorHandler is reflected in getErrorHandler.
  XmlRpcErrorHandler::setErrorHandler(&errors);
  EXPECT_EQ(&errors, XmlRpcErrorHandler::getErrorHandler());

  // Basic error check.
  XmlRpcUtil::error("Error!");
  EXPECT_EQ("Error!", errors.last_msg);
  errors.last_msg = "";

  // Error check with formatting.
  XmlRpcUtil::error("%d: I'm a teapot", 408);
  EXPECT_EQ("408: I'm a teapot", errors.last_msg);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
