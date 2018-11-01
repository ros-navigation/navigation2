// TestBase64Server.cpp : Simple XMLRPC server example. Usage: TestBase64Server serverPort
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif


#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdlib.h>


#include "xmlrpcpp/XmlRpc.h"
using namespace XmlRpc;


// The server
XmlRpcServer s;

// No arguments, result is Base64-encoded pngnow.png data.
class TestBase64 : public XmlRpcServerMethod
{
public:
  TestBase64(XmlRpcServer* s) : XmlRpcServerMethod("TestBase64", s) {}

  void execute(XmlRpcValue& params, XmlRpcValue& result)
  {
    std::ifstream infile("pngnow.png", std::ios::binary);
    if (infile.fail())
      infile.open("../pngnow.png", std::ios::binary);
    if (infile.fail())
      result = "Could not open file pngnow.png";
    else {

      XmlRpcValue::BinaryData& data = result;
      int n = 0;
      for (;; ++n) {
        char c = infile.get();
        if (infile.eof()) break;
        data.push_back(c);
      }
      std::cerr << "Read " << n << " bytes from pngnow.png\n";
    }
  }
} TestBase64(&s);    // This constructor registers the method with the server



int main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: TestBase64Server serverPort\n";
    return -1;
  }
  int port = atoi(argv[1]);

  //XmlRpc::setVerbosity(5);

  // Create the server socket on the specified port
  s.bindAndListen(port);

  // Wait for requests indefinitely
  s.work(-1.0);

  return 0;
}

