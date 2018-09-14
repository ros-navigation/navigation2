/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include "costmap_2d/array_parser.h"

using namespace costmap_2d;

TEST(array_parser, basic_operation)
{
  std::string error;
  std::vector<std::vector<float> > vvf;
  vvf = parseVVF( "[[1, 2.2], [.3, -4e4]]", error );
  EXPECT_EQ( 2, vvf.size() );
  EXPECT_EQ( 2, vvf[0].size() );
  EXPECT_EQ( 2, vvf[1].size() );
  EXPECT_EQ( 1.0f, vvf[0][0] );
  EXPECT_EQ( 2.2f, vvf[0][1] );
  EXPECT_EQ( 0.3f, vvf[1][0] );
  EXPECT_EQ( -40000.0f, vvf[1][1] );
  EXPECT_EQ( "", error );
}

TEST(array_parser, missing_open)
{
  std::string error;
  std::vector<std::vector<float> > vvf;
  vvf = parseVVF( "[1, 2.2], [.3, -4e4]]", error );
  EXPECT_FALSE( error == "" );
}

TEST(array_parser, missing_close)
{
  std::string error;
  std::vector<std::vector<float> > vvf;
  vvf = parseVVF( "[[1, 2.2], [.3, -4e4]", error );
  EXPECT_FALSE( error == "" );
}

TEST(array_parser, wrong_depth)
{
  std::string error;
  std::vector<std::vector<float> > vvf;
  vvf = parseVVF( "[1, 2.2], [.3, -4e4]", error );
  EXPECT_FALSE( error == "" );
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}

