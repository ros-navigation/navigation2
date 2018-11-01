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

#include <b64/encode.h>
#include <b64/decode.h>

#include <gtest/gtest.h>

// Test Data for a Base64 encode/decode test
class Base64TestData {
public:
  Base64TestData(std::vector<char> raw, std::string encoded)
    : raw(raw), encoded(encoded) {}

  std::vector<char> raw;
  std::string encoded;
};

class Base64Test : public ::testing::TestWithParam<Base64TestData> {};

TEST_P(Base64Test, Encode) {
  const std::vector<char> & data = GetParam().raw;

  std::stringstream is;
  is.write(&data[0], data.size());
  std::stringstream os;
  base64::encoder encoder;
  encoder.encode(is, os);

  std::string expected = GetParam().encoded;

  EXPECT_EQ(expected, os.str());
}

TEST_P(Base64Test, Decode) {
  const std::string& in = GetParam().encoded;
  const int encoded_size = in.length();

  // oversize our output vector (same method used by XmlRpcValue)
  std::vector<char> out;
  out.resize(encoded_size);

  base64::decoder decoder;
  const int size = decoder.decode(in.c_str(), encoded_size, &out[0]);
  ASSERT_LE(0, size);
  out.resize(size);

  const std::vector<char> & expected = GetParam().raw;

  EXPECT_EQ(expected, out);
}

INSTANTIATE_TEST_CASE_P(
    Multiline,
    Base64Test,
    ::testing::Values(
        Base64TestData({0}, "AA==\n"),
        Base64TestData({1, 2}, "AQI=\n"),
        Base64TestData({1, 2, 3}, "AQID\n"),
        Base64TestData(
            // clang-format off
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
             19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
             35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
             51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66,
             67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82,
             83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98,
             99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
             112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124,
             125, 126, 127, -128, -127, -126, -125, -124, -123, -122, -121,
             -120, -119, -118, -117, -116, -115, -114, -113, -112, -111, -110,
             -109, -108, -107, -106, -105, -104, -103, -102, -101, -100, -99,
             -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86,
             -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73,
             -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60,
             -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47,
             -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34,
             -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21,
             -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7,
             -6, -5, -4, -3, -2, -1},
            // clang-format on
            "AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gISIjJCUmJygpKissLS4vMD"
            "EyMzQ1\nNjc4OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fYG"
            "FiY2RlZmdoaWpr\nbG1ub3BxcnN0dXZ3eHl6e3x9fn+AgYKDhIWGh4iJiouMjY6PkJ"
            "GSk5SVlpeYmZqbnJ2en6Ch\noqOkpaanqKmqq6ytrq+wsbKztLW2t7i5uru8vb6/wM"
            "HCw8TFxsfIycrLzM3Oz9DR0tPU1dbX\n2Nna29zd3t/g4eLj5OXm5+jp6uvs7e7v8P"
            "Hy8/T19vf4+fr7/P3+/w==\n")));

class Base64ErrorData {
public:
  // TODO(future work): add error code representation here and check that error
  //                    codes are reported correctly.
  Base64ErrorData(std::string encoded)
    : encoded(encoded) {}

  std::string encoded;
};

class Base64ErrorTest : public ::testing::TestWithParam<Base64ErrorData> {};

TEST_P(Base64ErrorTest, DecodeErrors) {
  const std::string& in = GetParam().encoded;
  const int encoded_size = in.length();

  // oversize our output vector (same method used by XmlRpcValue)
  std::vector<char> out;
  out.resize(encoded_size);

  base64::decoder decoder;
  const int size = decoder.decode(in.c_str(), encoded_size, &out[0]);
  // Assert that size is greater or equal to 0, to make sure that the follow-up
  // resize will always succeed.
  ASSERT_LE(0, size);
  out.resize(size);

  // FIXME(future work): decode does not report error on garbage input
}

INSTANTIATE_TEST_CASE_P(
    Multiline,
    Base64ErrorTest,
    ::testing::Values(// Tests on incomplete data.
                      Base64ErrorData("="),
                      Base64ErrorData("A="),
                      Base64ErrorData("A"),
                      Base64ErrorData("AA"),
                      Base64ErrorData("AAA"),
                      Base64ErrorData("AA="),
                      // Tests with 4 bytes of good data but which does not
                      // terminate on the correct boundary.
                      Base64ErrorData("BBBBA="),
                      Base64ErrorData("BBBBA"),
                      Base64ErrorData("BBBBAA"),
                      Base64ErrorData("BBBBAAA"),
                      Base64ErrorData("BBBBAA="),
                      // Decode should succeed and do nothing on empty string.
                      Base64ErrorData(""),
                      // Character out of bounds for base64 encoding.
                      Base64ErrorData("<")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
