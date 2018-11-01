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

#include "xmlrpcpp/XmlRpc.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gtest/gtest.h>

// No arguments, result is "Hello".
class Hello : public XmlRpc::XmlRpcServerMethod
{
public:
  Hello(XmlRpc::XmlRpcServer* s) : XmlRpc::XmlRpcServerMethod("Hello", s) {}

  virtual ~Hello() {}

  void execute(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  boost::mutex hello_mutex;
};

class XmlRpcTest : public ::testing::Test
{
protected:
  XmlRpcTest();

  void work();

  virtual void SetUp();

  virtual void TearDown();

  // The server and its methods
  XmlRpc::XmlRpcServer s;
  Hello hello;

  // Server port number (for clients)
  int port;

  // Server thread
  bool server_done;
  boost::thread server_thread;
};
