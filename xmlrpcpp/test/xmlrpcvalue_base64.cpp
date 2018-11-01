#include <gtest/gtest.h>
#include "xmlrpcpp/XmlRpcValue.h"

#include <algorithm>
#include <string>
#include <sstream>

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace XmlRpc
{

  XmlRpcValue base64Value(std::string const & data)
  {
    return XmlRpcValue(const_cast<char *>(&data[0]), data.size());
  }

  XmlRpcValue fromXml(std::string const & data)
  {
    int offset = 0;
    return XmlRpcValue(data, &offset);
  }

  void removeNewlines(std::string & data)
  {
    data.erase(std::remove(data.begin(), data.end(), '\n'), data.end());
  }

  void assertEncodeDecode(std::string const & raw, std::string const & base64)
  {
    XmlRpcValue value  = base64Value(raw);
    std::string xml    ="<value><base64>" + base64 + "</base64></value>";
    XmlRpcValue parsed_value = fromXml(xml);
    std::string generated_xml = value.toXml();
    removeNewlines(generated_xml);

    std::stringstream buffer;
    value.write(buffer);
    std::string streamed = buffer.str();
    removeNewlines(streamed);
    ASSERT_EQ(streamed, base64);

    ASSERT_EQ(generated_xml, "<value><base64>" + base64 + "</base64></value>");
    ASSERT_EQ(parsed_value, value);
  }


  TEST(xmlrpcvalue_base64, empty_string)
  {
    assertEncodeDecode("", "");
  }

  TEST(xmlrpcvalue_base64, hello_world)
  {
    assertEncodeDecode("Hello World!", "SGVsbG8gV29ybGQh");
    assertEncodeDecode("Hello World!\n", "SGVsbG8gV29ybGQhCg==");
  }

  TEST(xmlrpcvalue_base64, random)
  {
    assertEncodeDecode(
      std::string("\261", 1),
      "sQ=="
    );
    assertEncodeDecode(
      std::string("\341\370", 2),
      "4fg="
    );
    assertEncodeDecode(
      std::string("\206\262J", 3),
      "hrJK"
    );
    assertEncodeDecode(
      std::string("|5Q%", 4),
      "fDVRJQ=="
    );
    assertEncodeDecode(
      std::string("5\220,+X", 5),
      "NZAsK1g="
    );
    assertEncodeDecode(
      std::string("\247\342\007M@\270", 6),
      "p+IHTUC4"
    );
    assertEncodeDecode(
      std::string("\012\247e\013;\232*", 7),
      "CqdlCzuaKg=="
    );
    assertEncodeDecode(
      std::string("U\374\336w\351-\2503", 8),
      "Vfzed+ktqDM="
    );
    assertEncodeDecode(
      std::string("\264\204`\310\001\306\253g\026", 9),
      "tIRgyAHGq2cW"
    );
    assertEncodeDecode(
      std::string("\224\307\217\336|J^\223\237v", 10),
      "lMeP3nxKXpOfdg=="
    );
    assertEncodeDecode(
      std::string("~%\305\024\264P)\206\224\247N", 11),
      "fiXFFLRQKYaUp04="
    );
    assertEncodeDecode(
      std::string("-4\355\215Q|\367\332j\013\027\006", 12),
      "LTTtjVF899pqCxcG"
    );
    assertEncodeDecode(
      std::string("\026Q\304\204\244s%\203\023?\364\320\005", 13),
      "FlHEhKRzJYMTP/TQBQ=="
    );
    assertEncodeDecode(
      std::string("G0\306B\251\351;mwM\312i(\000", 14),
      "RzDGQqnpO213TcppKAA="
    );
    assertEncodeDecode(
      std::string("\031kpC)[t\205\026\230\343S\367\016\263", 15),
      "GWtwQylbdIUWmONT9w6z"
    );
    assertEncodeDecode(
      std::string("B&\134a\013\246u^V\001\260a\247\231}U\3278\367=G\317g\020{\277\042\373", 28),
      "QiZcYQumdV5WAbBhp5l9Vdc49z1Hz2cQe78i+w=="
    );
    assertEncodeDecode(
      std::string("[>\303\210`\270o\023FJ\004r\360\361\357\2517\345\204|\245\267P \247V\251v\025", 29),
      "Wz7DiGC4bxNGSgRy8PHvqTflhHylt1Agp1apdhU="
    );
    assertEncodeDecode(
      std::string("\264\245\360=\273E%8\023\3138\227k\222WS\227uf\314\345Y\031\365Q\024\331.\210&", 30),
      "tKXwPbtFJTgTyziXa5JXU5d1ZszlWRn1URTZLogm"
    );
    assertEncodeDecode(
      std::string("II\036\326\353\232T\221/\021\3265F}\2647A)O\2302CLY?\367O\261E\246\332\376\333\230\343\326_\373\303\225\343\351\354[\326~\264\034\305\035V\227\034\331[\012\263Y", 58),
      "SUke1uuaVJEvEdY1Rn20N0EpT5gyQ0xZP/dPsUWm2v7bmOPWX/vDlePp7FvWfrQcxR1WlxzZWwqzWQ=="
    );
    assertEncodeDecode(
      std::string("\353\317\000-\333\230\2701\251\310&J\225\347+\221q\245tb\336\225\350\323\310\327*q\261&;\2448\033X\346\277\245GM\216Oz\365\025K1\341w\230\277\272*\365\343\204\376\334E", 59),
      "688ALduYuDGpyCZKlecrkXGldGLelejTyNcqcbEmO6Q4G1jmv6VHTY5PevUVSzHhd5i/uir144T+3EU="
    );
    assertEncodeDecode(
      std::string("\235\232\352\362\304\351n\244\227\220\202hh\226\346\271\042\021d\362c\267\260\310`\000\034\247\215\350\013;#2\204(^_\333w\2233\374\337\205\253y\372\352\305F-\200v\034\331\216\205\011", 60),
      "nZrq8sTpbqSXkIJoaJbmuSIRZPJjt7DIYAAcp43oCzsjMoQoXl/bd5Mz/N+Fq3n66sVGLX+AdhzZjoUJ"
    );
  }
}
