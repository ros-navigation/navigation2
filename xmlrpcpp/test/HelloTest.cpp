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
 * Loosely based on HelloServer.cpp and HelloClient.cpp by Chris Morley
 *
 */

#include "xmlrpcpp/XmlRpc.h"
#include "xmlrpcpp/XmlRpcClient.h"
#include "xmlrpcpp/XmlRpcServer.h"
#include "xmlrpcpp/XmlRpcServerMethod.h"

#include <iostream>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <functional>

#include <gtest/gtest.h>

using XmlRpc::XmlRpcServerMethod;
using XmlRpc::XmlRpcServer;
using XmlRpc::XmlRpcClient;
using XmlRpc::XmlRpcValue;

// No arguments, result is "Hello".
class Hello : public XmlRpcServerMethod
{
public:
  Hello(XmlRpcServer* s) : XmlRpcServerMethod("Hello", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    (void)params;
    result = "Hello";
  }

  std::string help()
  {
    return std::string("Say hello");
  }
};

// One argument is passed, result is "Hello, " + arg.
class HelloName : public XmlRpcServerMethod
{
public:
  HelloName(XmlRpcServer* s) : XmlRpcServerMethod("HelloName", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::string resultString = "Hello, ";
    resultString += std::string(params[0]);
    result = resultString;
  }
};

// A variable number of arguments are passed, all doubles, result is their sum.
class Sum : public XmlRpcServerMethod
{
public:
  Sum(XmlRpcServer* s) : XmlRpcServerMethod("Sum", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    int nArgs = params.size();
    double sum = 0.0;
    for (int i = 0; i < nArgs; ++i)
      sum += double(params[i]);
    result = sum;
  }
};

class XmlRpcTest : public ::testing::Test
{
protected:
  XmlRpcTest() : hello(&s), helloName(&s), sum(&s), port(0), done(false) {}

  void work()
  {
    while (!done)
    {
      s.work(0.1); // run the worker queue for 100ms
    }
  }

  virtual void SetUp()
  {
    // XmlRpc::setVerbosity(5);

    // Create the server socket. Passing 0 for the port number requests that
    // the OS randomly select an available port.
    s.bindAndListen(0);
    // Retrieve the assigned port number.
    port = s.get_port();

    // Enable introspection.
    s.enableIntrospection(true);

    // Start the worker thread.
    server_thread = boost::thread(boost::mem_fn(&XmlRpcTest::work), this);
  }

  virtual void TearDown()
  {
    // TODO(austin): determine if we need to do anything here to avoid
    // leaking resources
    done = true;
    server_thread.join();
    s.shutdown();
  }

  // The server and its methods
  XmlRpcServer s;
  Hello hello;
  HelloName helloName;
  Sum sum;

  // Server port number (for clients)
  int port;

  // Server thread
  bool done;
  boost::thread server_thread;
};

TEST_F(XmlRpcTest, Introspection)
{
  XmlRpcClient c("localhost", port);

  // Use introspection API to look up the supported methods
  XmlRpcValue noArgs, result;

  ASSERT_TRUE(c.execute("system.listMethods", noArgs, result));

  XmlRpcValue methods;
  methods[0] = "Hello";
  methods[1] = "HelloName";
  methods[2] = "Sum";
  methods[3] = "system.listMethods";
  methods[4] = "system.methodHelp";
  methods[5] = "system.multicall";
  EXPECT_EQ(result, methods);

  // Use introspection API to get the help string for the Hello method
  XmlRpcValue oneArg;
  oneArg[0] = "Hello";

  ASSERT_TRUE(c.execute("system.methodHelp", oneArg, result));

  EXPECT_EQ(result, XmlRpcValue("Say hello"));

  // Use introspection API to get the help string for the HelloName method
  // This should be the default help string, ie empty string.
  oneArg[0] = "HelloName";

  ASSERT_TRUE(c.execute("system.methodHelp", oneArg, result));

  EXPECT_EQ(result, XmlRpcValue(""));
}

TEST_F(XmlRpcTest, Hello)
{
  XmlRpcClient c("localhost", port);
  XmlRpcValue noArgs, result;

  // Call the Hello method
  ASSERT_TRUE(c.execute("Hello", noArgs, result));

  EXPECT_EQ(result, XmlRpcValue("Hello"));
}

TEST_F(XmlRpcTest, HelloURI)
{
  XmlRpcClient c("localhost", port, "/");
  XmlRpcValue noArgs, result;

  // Call the Hello method
  ASSERT_TRUE(c.execute("Hello", noArgs, result));

  EXPECT_EQ(result, XmlRpcValue("Hello"));
}

TEST_F(XmlRpcTest, HelloName)
{
  XmlRpcClient c("localhost", port);
  XmlRpcValue oneArg, result;

  // Call the HelloName method
  oneArg[0] = "Chris";
  ASSERT_TRUE(c.execute("HelloName", oneArg, result));

  EXPECT_EQ(result, XmlRpcValue("Hello, Chris"));
}

TEST_F(XmlRpcTest, Sum)
{
  XmlRpcClient c("localhost", port);
  XmlRpcValue result;

  // Add up an array of numbers
  XmlRpcValue numbers;
  numbers[0] = 33.33;
  numbers[1] = 112.57;
  numbers[2] = 76.1;
  EXPECT_EQ(numbers.size(), 3);

  ASSERT_TRUE(c.execute("Sum", numbers, result));
  EXPECT_DOUBLE_EQ(double(result), 222.0);

  // Test the "no such method" fault
  ASSERT_TRUE(c.execute("NoSuchMethod", numbers, result));
  EXPECT_TRUE(c.isFault());

  XmlRpcValue fault;
  fault["faultCode"] = -1;
  fault["faultString"] = "NoSuchMethod: unknown method name";
  EXPECT_EQ(result, fault);
}

TEST_F(XmlRpcTest, Multicall)
{
  XmlRpcClient c("localhost", port);
  XmlRpcValue result;

  // Test the multicall method. It accepts one arg, an array of structs
  XmlRpcValue multicall, expected_result;
  multicall[0][0]["methodName"] = "Sum";
  multicall[0][0]["params"][0] = 5.0;
  multicall[0][0]["params"][1] = 9.0;
  expected_result[0][0] = 14.0;

  multicall[0][1]["methodName"] = "NoSuchMethod";
  multicall[0][1]["params"][0] = "";
  expected_result[1]["faultCode"] = -1;
  expected_result[1]["faultString"] = "NoSuchMethod: unknown method name";

  multicall[0][2]["methodName"] = "Sum";
  // Missing params
  expected_result[2]["faultCode"] = -1;
  expected_result[2]["faultString"] = "system.multicall: Invalid argument "
                                      "(expected a struct with members "
                                      "methodName and params)";

  multicall[0][3]["methodName"] = "Sum";
  multicall[0][3]["params"][0] = 10.5;
  multicall[0][3]["params"][1] = 12.5;
  expected_result[3][0] = 23.0;

  ASSERT_TRUE(c.execute("system.multicall", multicall, result));
  EXPECT_EQ(result, expected_result);
  EXPECT_EQ(result.toXml(), expected_result.toXml());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
