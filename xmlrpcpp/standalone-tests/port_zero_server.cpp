// HelloServer.cpp : Simple XMLRPC server example. Usage: HelloServer serverPort
//
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <stdlib.h>

using namespace XmlRpc;

// The server
XmlRpcServer s;

// No arguments, result is "Hello".
class Hello : public XmlRpcServerMethod
{
public:
  Hello(XmlRpcServer* s) : XmlRpcServerMethod("Hello", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    result = "Hello";
  }

  std::string help() { return std::string("Say hello"); }

} hello(&s);    // This constructor registers the method with the server


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
} helloName(&s);


// A variable number of arguments are passed, all doubles, result is their sum.
class Sum : public XmlRpcServerMethod
{
public:
  Sum(XmlRpcServer* s) : XmlRpcServerMethod("Sum", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    int nArgs = params.size();
    double sum = 0.0;
    for (int i=0; i<nArgs; ++i)
      sum += double(params[i]);
    result = sum;
  }
} sum(&s);


int main(int argc, char* argv[])
{
  XmlRpc::setVerbosity(5);

  // Create the server socket on the specified port
  s.bindAndListen(0);

  // Enable introspection
  s.enableIntrospection(true);

  // Wait for requests indefinitely
  s.work(-1.0);

  return 0;
}

