// Validator.cpp : XMLRPC server based on the compliancy test at validator.xmlrpc.com.
//
#include "xmlrpcpp/XmlRpc.h"
using namespace XmlRpc;

#include <iostream>


XmlRpcServer s;


// One argument is passed, an array of structs, each with a member named curly with 
// an integer value. Return the sum of those values.

class ArrayOfStructsTest : public XmlRpcServerMethod
{
public:
  ArrayOfStructsTest(XmlRpcServer* s) : XmlRpcServerMethod("validator1.arrayOfStructsTest", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "ArrayOfStructsTest\n";
    XmlRpcValue& arg1 = params[0];
    int n = arg1.size(), sum = 0;
    for (int i=0; i<n; ++i) 
      sum += int(arg1[i]["curly"]);

    result = sum;
  }
} arrayOfStructsTest(&s);


// This handler takes a single parameter, a string, that contains any number of predefined 
// entities, namely <, >, &, ' and ".
// The handler must return a struct that contains five fields, all numbers: ctLeftAngleBrackets, 
// ctRightAngleBrackets, ctAmpersands, ctApostrophes, ctQuotes. 
// To validate, the numbers must be correct.

class CountTheEntities : public XmlRpcServerMethod
{
public:
  CountTheEntities(XmlRpcServer* s) : XmlRpcServerMethod("validator1.countTheEntities", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "CountTheEntities\n";
    std::string& arg = params[0];
    int ctLeftAngleBrackets = 0;
    int ctRightAngleBrackets = 0;
    int ctAmpersands = 0;
    int ctApostrophes = 0;
    int ctQuotes = 0;

    int n = int(arg.length());
    for (int i=0; i<n; ++i)
      switch (arg[i])
      {
        case '<': ++ctLeftAngleBrackets; break;
        case '>': ++ctRightAngleBrackets; break;
        case '&': ++ctAmpersands; break;
        case '\'': ++ctApostrophes; break;
        case '\"': ++ctQuotes; break;
      }

    result["ctLeftAngleBrackets"] = ctLeftAngleBrackets;
    result["ctRightAngleBrackets"] = ctRightAngleBrackets;
    result["ctAmpersands"] = ctAmpersands;
    result["ctApostrophes"] = ctApostrophes;
    result["ctQuotes"] = ctQuotes;
  }
} countTheEntities(&s);



// This handler takes a single parameter, a struct, containing at least three elements 
// named moe, larry and curly, all <i4>s. Your handler must add the three numbers and 
// return the result.

class EasyStructTest : public XmlRpcServerMethod
{
public:
  EasyStructTest(XmlRpcServer* s) : XmlRpcServerMethod("validator1.easyStructTest", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "EasyStructTest\n";
    XmlRpcValue& arg1 = params[0];
    int sum = int(arg1["moe"]) + int(arg1["larry"]) + int(arg1["curly"]);
    result = sum;
  }
} easyStructTest(&s);


// This handler takes a single parameter, a struct. Your handler must return the struct.

class EchoStructTest : public XmlRpcServerMethod
{
public:
  EchoStructTest(XmlRpcServer* s) : XmlRpcServerMethod("validator1.echoStructTest", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "EchoStructTest\n";
    result = params[0];
  }
} echoStructTest(&s);



// This handler takes six parameters, and returns an array containing all the parameters.

class ManyTypesTest : public XmlRpcServerMethod
{
public:
  ManyTypesTest(XmlRpcServer* s) : XmlRpcServerMethod("validator1.manyTypesTest", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "ManyTypesTest\n";
    result = params;
  }
} manyTypesTest(&s);



// This handler takes a single parameter, which is an array containing between 100 and 
// 200 elements. Each of the items is a string, your handler must return a string 
// containing the concatenated text of the first and last elements.


class ModerateSizeArrayCheck : public XmlRpcServerMethod
{
public:
  ModerateSizeArrayCheck(XmlRpcServer* s) : XmlRpcServerMethod("validator1.moderateSizeArrayCheck", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "ModerateSizeArrayCheck\n";
    std::string s = params[0][0];
    s += std::string(params[0][params[0].size()-1]);
    result = s;
  }
} moderateSizeArrayCheck(&s);


// This handler takes a single parameter, a struct, that models a daily calendar.
// At the top level, there is one struct for each year. Each year is broken down
// into months, and months into days. Most of the days are empty in the struct
// you receive, but the entry for April 1, 2000 contains a least three elements
// named moe, larry and curly, all <i4>s. Your handler must add the three numbers
// and return the result.

class NestedStructTest : public XmlRpcServerMethod
{
public:
  NestedStructTest(XmlRpcServer* s) : XmlRpcServerMethod("validator1.nestedStructTest", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "NestedStructTest\n";
    XmlRpcValue& dayStruct = params[0]["2000"]["04"]["01"];
    int sum = int(dayStruct["moe"]) + int(dayStruct["larry"]) + int(dayStruct["curly"]);
    result = sum;
  }
} nestedStructTest(&s);



// This handler takes one parameter, and returns a struct containing three elements, 
// times10, times100 and times1000, the result of multiplying the number by 10, 100 and 1000.

class SimpleStructReturnTest : public XmlRpcServerMethod
{
public:
  SimpleStructReturnTest(XmlRpcServer* s) : XmlRpcServerMethod("validator1.simpleStructReturnTest", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::cerr << "SimpleStructReturnTest\n";
    int n = params[0];
    result["times10"] = n * 10;
    result["times100"] = n * 100;
    result["times1000"] = n * 1000;
  }
} simpleStructReturnTest(&s);



int main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: Validator port\n";
    return -1;
  }
  int port = atoi(argv[1]);

  XmlRpc::setVerbosity(5);

  // Create the server socket on the specified port
  s.bindAndListen(port);

  // Wait for requests indefinitely
  s.work(-1.0);

  return 0;
}

