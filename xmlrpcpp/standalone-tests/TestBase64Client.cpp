// TestBase64Client.cpp : A simple xmlrpc client that returns a png file
// encoded as base64 data to the client.
//
// Usage: TestBase64Client serverHost serverPort outputfile
// Requests a png file from the specified server and saves it in outputfile.
// Link against xmlrpc lib and whatever socket libs your system needs (ws2_32.lib on windows)

#include "xmlrpcpp/XmlRpc.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>

using namespace XmlRpc;

int main(int argc, char* argv[])
{
  if (argc != 4) {
    std::cerr << "Usage: TestBase64Client serverHost serverPort outputFile\n";
    return -1;
  }
  int port = atoi(argv[2]);

  //XmlRpc::setVerbosity(5);
  XmlRpcClient c(argv[1], port);

  XmlRpcValue noArgs, result;
  if (c.execute("TestBase64", noArgs, result))
  {
    const XmlRpcValue::BinaryData& data = result;
    std::ofstream outfile(argv[3], std::ios::binary | std::ios::trunc);
    if (outfile.fail())
      std::cerr << "Error opening " << argv[3] << " for output.\n";
    else
    {
      int n = int(data.size());
      for (int i=0; i<n; ++i)
        outfile << data[i];
    }
  }
  else
    std::cout << "Error calling 'TestBase64'\n\n";

  return 0;
}

