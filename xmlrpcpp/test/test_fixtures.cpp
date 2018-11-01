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
 * Loosley based on HelloServer.cpp by Chris Morley
 *
 */

#include "test_fixtures.h"
// No arguments, result is "Hello".

using namespace XmlRpc;

void Hello::execute(XmlRpcValue& params, XmlRpcValue& result)
{
  (void)params;
  boost::unique_lock<boost::mutex> lock(hello_mutex);
  result = "Hello";
}

XmlRpcTest::XmlRpcTest() : hello(&s), port(0), server_done(false) {}

void XmlRpcTest::work()
{
  while (!server_done)
  {
    s.work(0.1); // run the worker queue for 100ms
  }
}

void XmlRpcTest::SetUp()
{
  // XmlRpc::setVerbosity(5);

  // Create the server socket on the specified port
  s.bindAndListen(0);
  port = s.get_port();

  // Enable introspection
  s.enableIntrospection(true);

  // Start the worker thread
  server_thread = boost::thread(boost::mem_fn(&XmlRpcTest::work), this);
}

void XmlRpcTest::TearDown()
{
  // TODO(austin): determine if we need to do anything here to avoid
  // leaking resources
  server_done = true;
  if (server_thread.joinable())
  {
    server_thread.join();
  }
  s.shutdown();

  // Reset verbosity in case a test raises the verbosity.
  XmlRpc::setVerbosity(0);
}
