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
 * Loosely based on the original TestXml.cpp by Chris Morley
 *
 */

// TestXml.cpp : Test XML encoding and decoding.
// The characters <>&'" are illegal in xml and must be encoded.

#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers

#include <iostream>
// If you are using MSVC++6, you should update <string> to fix
// BUG: getline Template Function Reads Extra Character
#include <string>
#include <stdlib.h>

#include "xmlrpcpp/XmlRpcUtil.h"

#include <gtest/gtest.h>

using namespace XmlRpc;

TEST(XmlRpc, BasicXml) {
  // Basic tests
  std::string empty;
  EXPECT_EQ(empty, XmlRpcUtil::xmlEncode(empty));
  EXPECT_EQ(empty, XmlRpcUtil::xmlDecode(empty));
  EXPECT_EQ(empty, XmlRpcUtil::xmlEncode(""));
  EXPECT_EQ(empty, XmlRpcUtil::xmlDecode(""));

  std::string raw("<>&'\"");
  EXPECT_EQ(XmlRpcUtil::xmlDecode(XmlRpcUtil::xmlEncode(raw)), raw);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
