// TestValues.cpp : Test XML encoding and decoding of XmlRpcValues.

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include "xmlrpcpp/XmlRpcValue.h"


#include <assert.h>
#include <iostream>


using namespace XmlRpc;


void testBoolean()
{
  XmlRpcValue booleanFalse(false);
  XmlRpcValue booleanTrue(true);
  int offset = 0;
  XmlRpcValue booleanFalseXml("<value><boolean>0</boolean></value>", &offset);
  offset = 0;
  XmlRpcValue booleanTrueXml("<value><boolean>1</boolean></value>", &offset);
  assert(booleanFalse != booleanTrue);
  assert(booleanFalse == booleanFalseXml);
  assert(booleanFalse == booleanFalseXml);
  if (booleanFalse)
    assert(false);

  if (booleanTrue)
    assert( ! false);
  else
    assert(false);
}

// Int
void testInt()
{
  XmlRpcValue int0(0);
  XmlRpcValue int1(1);
  XmlRpcValue int10(10);
  XmlRpcValue int_1(-1);
  int offset = 0;
  XmlRpcValue int0Xml("<value><int>0</int></value>", &offset);
  offset = 0;
  XmlRpcValue int9Xml("<value><i4>9</i4></value>", &offset);
  assert(int0 == int0Xml);
  assert(int(int10) - int(int1) == int(int9Xml));
  assert(9 == int(int9Xml));
  assert(int(int10) + int(int_1) == int(int9Xml));
}

void testDouble()
{
  // Double
  XmlRpcValue d(43.7);
  int offset = 0;
  XmlRpcValue dXml("<value><double>56.3</double></value>", &offset);
  assert(double(d) + double(dXml) == 100.0);  // questionable practice...
}

void testString()
{
  // String
  XmlRpcValue s("Now is the time <&");
  char csxml[] = "<value><string>Now is the time &lt;&amp;</string></value>";
  std::string ssxml = csxml;
  int offset = 0;
  XmlRpcValue vscXml(csxml, &offset);
  offset = 0;
  XmlRpcValue vssXml(ssxml, &offset);
  assert(s == vscXml);
  assert(s == vssXml);
  offset = 0;
  XmlRpcValue fromXml(vssXml.toXml(), &offset);
  assert(s == fromXml);

  // Empty or blank strings with no <string> tags
  std::string emptyStringXml("<value></value>");
  offset = 0;
  XmlRpcValue emptyStringVal1(emptyStringXml, &offset);
  XmlRpcValue emptyStringVal2("");
  assert(emptyStringVal1 == emptyStringVal2);

  emptyStringXml = "<value>  </value>";
  offset = 0;
  XmlRpcValue blankStringVal(emptyStringXml, &offset);
  assert(std::string(blankStringVal) == "  ");
}


void testDateTime()
{
  // DateTime
  int offset = 0;
  XmlRpcValue dateTime("<value><dateTime.iso8601>19040101T03:12:35</dateTime.iso8601></value>", &offset);
  struct tm &t = dateTime;
  assert(t.tm_year == 1904 && t.tm_min == 12);
}


void testArray(XmlRpcValue const& d)
{
  // Array
  XmlRpcValue a;
  a.setSize(4);
  a[0] = 1;
  a[1] = std::string("two");
  a[2] = 43.7;
  a[3] = "four";
  assert(int(a[0]) == 1);
  assert(a[2] == d);

  char csaXml[] =
    "<value><array>\n"
    "  <data>\n"
    "    <value><i4>1</i4></value> \n"
    "    <value> <string>two</string></value>\n"
    "    <value><double>43.7</double></value>\n"
    "    <value>four</value>\n"
    "  </data>\n"
    "</array></value>";
    
  int offset = 0;
  XmlRpcValue aXml(csaXml, &offset);
  assert(a == aXml);
}

void testStruct()
{
  // Struct
  XmlRpcValue struct1;
  struct1["i4"] = 1;
  struct1["str"] = "two";
  struct1["d"] = 43.7;

  XmlRpcValue a;
  a.setSize(4);
  a[0] = 1;
  a[1] = std::string("two");
  a[2] = 43.7;
  a[3] = "four";

  assert(struct1["d"] == a[2]);

  char csStructXml[] =
    "<value><struct>\n"
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
  assert(struct1 == structXml);

  XmlRpcValue astruct;
  astruct["array"] = a;
  assert(astruct["array"][2] == struct1["d"]);

  for (int i=0; i<10; i++) {
    XmlRpcValue Event;
    Event["Name"] = "string";

    Event.clear();

    const int NELMTS = 100;
    int ii;

    for (ii=0; ii< NELMTS; ++ii) {
      char buf[40];
      sprintf(buf,"%d", ii);
      Event[buf] = buf;
    }

    Event.clear();

    for (ii=0; ii< NELMTS; ++ii) {
      char buf[40];
      sprintf(buf,"%d", ii);
      if (ii != NELMTS/2)
        Event[buf] = ii;
      else
        for (int jj=0; jj< NELMTS; ++jj) {
          char bufj[40];
          sprintf(bufj,"%d", jj);
          Event[buf][bufj] = bufj;
        }
    }

    for (ii=0; ii< NELMTS; ++ii) {
      char buf[40];
      sprintf(buf,"%d", ii);
      if (ii != NELMTS/2)
        assert(Event[buf] == XmlRpcValue(ii));
      else
        assert(Event[buf].size() == NELMTS);
    }
  }
}



int main(int argc, char* argv[])
{
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );

  testBoolean();
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );

  testInt();
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );


  testDouble();
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );


  testString();
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );


  testDateTime();
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );


  testArray(43.7);
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );


  testStruct();
  _CrtDumpMemoryLeaks();
  _CrtCheckMemory( );

  return 0;
}
