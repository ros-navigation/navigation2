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
 * Loosely based on the original TestValues.cpp by Chris Morley
 *
 */

// TestValues.cpp : Test XML encoding and decoding of XmlRpcValues.

#include <stdlib.h>
#include <string>

#include "xmlrpcpp/XmlRpcValue.h"
#include "xmlrpcpp/XmlRpcException.h"

#include <gtest/gtest.h>

using namespace XmlRpc;

TEST(XmlRpc, Bool) {
  XmlRpcValue v(bool(false));

  EXPECT_EQ("<value><boolean>0</boolean></value>", v.toXml());

  XmlRpcValue v2;
  v2[0] = int(1);
  v2[1] = std::string();
  v2[2] = XmlRpcValue(false);
  EXPECT_EQ("<value><array><data><value><i4>1</i4></value><value></"
            "value><value><boolean>0</boolean></value></data></array></value>",
            v2.toXml());
}

TEST(XmlRpc, testBoolean) {
  XmlRpcValue booleanFalse(false);
  XmlRpcValue booleanTrue(true);
  int offset = 0;
  XmlRpcValue booleanFalseXml("<value><boolean>0</boolean></value>", &offset);
  offset = 0;
  XmlRpcValue booleanTrueXml("<value><boolean>1</boolean></value>", &offset);
  EXPECT_NE(booleanFalse, booleanTrue);
  EXPECT_EQ(booleanFalse, booleanFalseXml);
  EXPECT_NE(booleanFalse, booleanTrueXml);

  EXPECT_FALSE(bool(booleanFalse));

  EXPECT_TRUE(bool(booleanTrue));

  // Test stream operator.
  std::stringstream ss;
  ss << booleanFalse;
  EXPECT_EQ("0", ss.str());

  std::stringstream ss2;
  ss2 << booleanTrue;
  EXPECT_EQ("1", ss2.str());
}

// Int
TEST(XmlRpc, testInt) {
  XmlRpcValue int0(0);
  ASSERT_EQ(XmlRpcValue::TypeInt, int0.getType());

  XmlRpcValue int1(1);
  ASSERT_EQ(XmlRpcValue::TypeInt, int1.getType());

  XmlRpcValue int10(10);
  ASSERT_EQ(XmlRpcValue::TypeInt, int10.getType());

  XmlRpcValue int_1(-1);
  ASSERT_EQ(XmlRpcValue::TypeInt, int_1.getType());

  int offset = 0;
  XmlRpcValue int0Xml("<value><int>0</int></value>", &offset);
  ASSERT_EQ(XmlRpcValue::TypeInt, int0Xml.getType());
  EXPECT_EQ(0, int(int0Xml));

  offset = 0;
  XmlRpcValue int9Xml("<value><i4>9</i4></value>", &offset);
  ASSERT_EQ(XmlRpcValue::TypeInt, int9Xml.getType());
  EXPECT_EQ(9, int(int9Xml));

  EXPECT_EQ(int0, int0Xml);
  EXPECT_EQ(int(int10) - int(int1), int(int9Xml));
  EXPECT_EQ(9, int(int9Xml));
  EXPECT_EQ(int(int10) + int(int_1), int(int9Xml));

  // Test stream operator.
  std::stringstream ss;
  ss << int9Xml;
  EXPECT_EQ("9", ss.str());
}

TEST(XmlRpc, testDouble) {
  // Double
  XmlRpcValue d(43.7);
  ASSERT_EQ(XmlRpcValue::TypeDouble, d.getType());
  EXPECT_EQ("<value><double>43.700000000000003</double></value>", d.toXml());
  EXPECT_DOUBLE_EQ(43.7, double(d));

  int offset = 0;
  XmlRpcValue dXml("<value><double>56.3</double></value>", &offset);
  ASSERT_EQ(XmlRpcValue::TypeDouble, dXml.getType());
  EXPECT_DOUBLE_EQ(56.3, double(dXml));

  EXPECT_DOUBLE_EQ(100.0, double(d) + double(dXml));

  // Test stream operator.
  std::stringstream ss;
  ss << d;
  EXPECT_EQ("43.7", ss.str());
}

TEST(XmlRpc, testString) {
  // String
  XmlRpcValue s("Now is the time <&");
  ASSERT_EQ(XmlRpcValue::TypeString, s.getType());
  EXPECT_EQ(18, s.size());
  EXPECT_EQ("<value>Now is the time &lt;&amp;</value>", s.toXml());

  char csxml[] = "<value><string>Now is the time &lt;&amp;</string></value>";
  std::string ssxml = csxml;

  int offset = 0;
  XmlRpcValue vscXml(csxml, &offset);
  EXPECT_EQ(s, vscXml);

  offset = 0;
  XmlRpcValue vssXml(ssxml, &offset);
  EXPECT_EQ(s, vssXml);

  offset = 0;
  XmlRpcValue fromXml(vssXml.toXml(), &offset);
  EXPECT_EQ(s, fromXml);

  // Empty or blank strings with no <string> tags
  std::string emptyStringXml("<value></value>");
  offset = 0;
  XmlRpcValue emptyStringVal1(emptyStringXml, &offset);
  XmlRpcValue emptyStringVal2("");
  EXPECT_EQ(emptyStringVal1, emptyStringVal2);

  emptyStringXml = "<value>  </value>";
  offset = 0;
  XmlRpcValue blankStringVal(emptyStringXml, &offset);
  EXPECT_EQ(std::string(blankStringVal), "  ");

  // Implicitly initialized string.
  XmlRpcValue s2;
  std::string tmp = s2;
  EXPECT_EQ("", tmp);
  EXPECT_EQ("", (std::string)s2);

  // Test stream operator.
  std::stringstream ss;
  ss << s;
  EXPECT_EQ("Now is the time <&", ss.str());
}

TEST(XmlRpc, testDateTime) {
  // DateTime
  int offset = 0;
  // Construct from XML
  XmlRpcValue dateTime(
      "<value><dateTime.iso8601>19040503T03:12:35</dateTime.iso8601></value>",
      &offset);
  ASSERT_EQ(XmlRpcValue::TypeDateTime, dateTime.getType());
  struct tm t = dateTime;
  EXPECT_EQ(t.tm_year, 1904);
  EXPECT_EQ(t.tm_min, 12);
  EXPECT_EQ(t.tm_sec, 35);
  EXPECT_EQ(t.tm_hour, 3);
  EXPECT_EQ(t.tm_mday, 3);
  EXPECT_EQ(t.tm_mon, 5);

  EXPECT_EQ(
      "<value><dateTime.iso8601>19040503T03:12:35</dateTime.iso8601></value>",
      dateTime.toXml());

  // Constructor from struct tm.
  XmlRpcValue dateTime2(&t);
  ASSERT_EQ(XmlRpcValue::TypeDateTime, dateTime2.getType());
  t = dateTime2;
  EXPECT_EQ(t.tm_year, 1904);
  EXPECT_EQ(t.tm_min, 12);

  // Implicit initialization by cast.
  XmlRpcValue dateTime3;
  t = dateTime3;
  ASSERT_EQ(XmlRpcValue::TypeDateTime, dateTime3.getType());
  EXPECT_EQ(t.tm_year, 0);
  EXPECT_EQ(t.tm_min, 0);

  // Test stream operator.
  std::stringstream ss;
  ss << dateTime;
  EXPECT_EQ("19040503T03:12:35", ss.str());

  // Tests for DateTime equality operator
  EXPECT_EQ(dateTime, dateTime2);

  // Copy operator
  dateTime3 = dateTime;
  EXPECT_EQ(dateTime, dateTime3);

  // Explicit mutable reference into dateTime2 so that we can modify it to make
  // it not equal to dateTime in various ways.
  t = dateTime;
  struct tm& tm2 = dateTime2;
  // Seconds not equal.
  tm2.tm_sec = 0;
  EXPECT_NE(dateTime, dateTime2);
  tm2 = t;
  ASSERT_EQ(dateTime, dateTime2);

  // Minutes not equal.
  tm2.tm_min = 0;
  EXPECT_NE(dateTime, dateTime2);
  tm2 = t;
  ASSERT_EQ(dateTime, dateTime2);

  // Hours not equal.
  tm2.tm_hour = 0;
  EXPECT_NE(dateTime, dateTime2);
  tm2 = t;
  ASSERT_EQ(dateTime, dateTime2);

  // Day not equal.
  tm2.tm_mday = 1;
  EXPECT_NE(dateTime, dateTime2);
  tm2 = t;
  ASSERT_EQ(dateTime, dateTime2);

  // Month not equal.
  tm2.tm_mon = 1;
  EXPECT_NE(dateTime, dateTime2);
  tm2 = t;
  ASSERT_EQ(dateTime, dateTime2);

  // Year not equal.
  tm2.tm_year = 1988;
  EXPECT_NE(dateTime, dateTime2);
}

TEST(XmlRpc, testArray) {
  XmlRpcValue d(43.7);
  // Array
  XmlRpcValue a;
  a.setSize(4);
  a[0] = 1;
  a[1] = std::string("two");
  a[2] = 43.7;
  a[3] = "four";
  EXPECT_EQ(XmlRpcValue::TypeArray, a.getType());
  EXPECT_EQ(int(a[0]), 1);
  EXPECT_EQ(a[2], d);

  char csaXml[] = "<value><array>\n"
                  "  <data>\n"
                  "    <value><i4>1</i4></value> \n"
                  "    <value> <string>two</string></value>\n"
                  "    <value><double>43.7</double></value>\n"
                  "    <value>four</value>\n"
                  "  </data>\n"
                  "</array></value>";

  int offset = 0;
  XmlRpcValue aXml(csaXml, &offset);
  EXPECT_EQ(a, aXml);

  // Array copy works
  XmlRpcValue copy(a);
  ASSERT_EQ(a.getType(), copy.getType());
  ASSERT_EQ(a.size(), copy.size());
  for (int i = 0; i < 3; i++) {
    EXPECT_EQ(a[i], copy[i]);
  }
  // Test that comparison operator works.
  EXPECT_EQ(a, copy);

  // Test that comparison for unlike types returns false.
  EXPECT_NE(a, d);

  // Test stream operator.
  std::stringstream ss;
  ss << a;
  EXPECT_EQ("{1,two,43.7,four}", ss.str());
}

TEST(XmlRpc, testStruct) {
  // Struct
  XmlRpcValue struct1;
  struct1["i4"] = 1;
  struct1["str"] = "two";
  struct1["d"] = 43.7;
  EXPECT_EQ(3, struct1.size());
  EXPECT_EQ(XmlRpcValue::TypeStruct, struct1.getType());
  EXPECT_TRUE(struct1.hasMember("i4"));
  EXPECT_FALSE(struct1.hasMember("nowhere"));

  // Test stream operator.
  std::stringstream ss;
  ss << struct1;
  EXPECT_EQ("[d:43.7,i4:1,str:two]", ss.str());

  XmlRpcValue a;
  a.setSize(4);
  a[0] = 1;
  a[1] = std::string("two");
  a[2] = 43.7;
  a[3] = "four";

  EXPECT_EQ(struct1["d"], a[2]);

  char csStructXml[] = "<value><struct>\n"
                       "  <member>\n"
                       "    <name>i4</name> \n"
                       "    <value><i4>1</i4></value> \n"
                       "  </member>\n"
                       "  <member>\n"
                       "    <name>d</name> \n"
                       "    <value><double>43.7</double></value>\n"
                       "  </member>\n"
                       "  <member>\n"
                       "    <name>str</name> \n"
                       "    <value> <string>two</string></value>\n"
                       "  </member>\n"
                       "</struct></value>";

  int offset = 0;
  XmlRpcValue structXml(csStructXml, &offset);
  EXPECT_EQ(struct1, structXml);

  for (XmlRpcValue::iterator itr = struct1.begin(); itr != struct1.end();
       itr++) {
  }

  XmlRpcValue astruct;
  astruct["array"] = a;
  EXPECT_EQ(astruct["array"][2], struct1["d"]);

  for (int i = 0; i < 10; i++) {
    XmlRpcValue Event;
    Event["Name"] = "string";

    Event.clear();

    const int NELMTS = 100;
    int ii;

    for (ii = 0; ii < NELMTS; ++ii) {
      char buf[40];
      sprintf(buf, "%d", ii);
      Event[std::string(buf)] = buf;
    }

    Event.clear();

    for (ii = 0; ii < NELMTS; ++ii) {
      char buf[40];
      sprintf(buf, "%d", ii);
      if (ii != NELMTS / 2)
        Event[std::string(buf)] = ii;
      else
        for (int jj = 0; jj < NELMTS; ++jj) {
          char bufj[40];
          sprintf(bufj, "%d", jj);
          Event[std::string(buf)][std::string(bufj)] = bufj;
        }
    }

    for (ii = 0; ii < NELMTS; ++ii) {
      char buf[40];
      sprintf(buf, "%d", ii);
      if (ii != NELMTS / 2)
        EXPECT_EQ(Event[std::string(buf)], XmlRpcValue(ii));
      else
        EXPECT_EQ(Event[std::string(buf)].size(), NELMTS);
    }
  }
}

TEST(XmlRpc, base64) {
  char data[] = {1, 2};
  XmlRpcValue bin(data, 2);

  EXPECT_EQ(XmlRpcValue::TypeBase64, bin.getType());
  EXPECT_EQ(2, bin.size());

  XmlRpcValue::BinaryData d = bin;
  EXPECT_EQ(d[0], 1);
  EXPECT_EQ(d[1], 2);

  EXPECT_EQ("<value><base64>AQI=\n</base64></value>", bin.toXml());

  // Test stream operator.
  std::stringstream ss;
  ss << bin;
  EXPECT_EQ("AQI=\n", ss.str());

  // Constructor from XML
  int offset = 0;
  XmlRpcValue bin2("<value><base64>AQI=</base64></value>", &offset);
  EXPECT_EQ(XmlRpcValue::TypeBase64, bin2.getType());
  EXPECT_EQ(2, bin2.size());

  d = bin2;
  EXPECT_EQ(d[0], 1);
  EXPECT_EQ(d[1], 2);

  EXPECT_EQ(bin, bin2);

  // Implicit initialization.
  XmlRpcValue bin3;
  d = bin3;
  EXPECT_EQ(XmlRpcValue::TypeBase64, bin3.getType());
  EXPECT_EQ(0, bin3.size());
  EXPECT_EQ(0u, d.size());

  // Copy operator
  XmlRpcValue bin4;
  bin4 = bin;

  EXPECT_EQ(XmlRpcValue::TypeBase64, bin4.getType());
  EXPECT_EQ(2, bin4.size());

  d = bin4;
  EXPECT_EQ(d[0], 1);
  EXPECT_EQ(d[1], 2);
}

TEST(XmpRpc, errors) {
  // Value is initially invalid.
  XmlRpcValue value;
  EXPECT_FALSE(value.valid());
  EXPECT_EQ("", value.toXml());

  // Implicit bool initialization.
  bool v = (bool)value;
  EXPECT_FALSE(v);

  // Conversions to other types should now throw an XmlRpcException.
  EXPECT_THROW((void)(int)value, XmlRpcException);
  EXPECT_THROW(value[0], XmlRpcException);
  EXPECT_THROW(value["bar"], XmlRpcException);

  // Tests on const objects.
  const XmlRpcValue& ref = value;
  EXPECT_THROW(ref[0], XmlRpcException);
  // TODO(austin): is this really allowed on non-struct objects?
  EXPECT_FALSE(ref.hasMember("bar"));

  // Check that the exception that is thrown is populated correctly.
  try {
    (void)int(value);
  } catch (XmlRpcException& e) {
    EXPECT_EQ("type error", e.getMessage());
    EXPECT_EQ(-1, e.getCode());
  }

  // size() on bool should throw.
  EXPECT_THROW(value.size(), XmlRpcException);

  // Clear should result in invalid again.
  value.clear();
  EXPECT_FALSE(value.valid());
}

TEST(XmlRpc, int_errors) {
  XmlRpcValue value;
  // Implicit int initialization.
  int v = (int)value;
  EXPECT_EQ(0, v);
  EXPECT_EQ(0, (int)value);

  // Conversion to other types should now thrown an exception.
  EXPECT_THROW((void)(bool)value, XmlRpcException);
}

TEST(XmlRpc, array_errors) {
  XmlRpcValue value;
  // Implicit array creation.
  int v = value[0];
  EXPECT_EQ(0, v);
  EXPECT_THROW((void)(bool)value, XmlRpcException);
  EXPECT_EQ(1, value.size());

  // Access on a non-const array should implicitly create another element.
  EXPECT_EQ(0, (int)value[1]);
  EXPECT_EQ(2, value.size());

  // Access to a const array should throw an exception if the index is out of
  // bounds.
  const XmlRpcValue& ref = value;
  EXPECT_THROW(ref[2], XmlRpcException);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
