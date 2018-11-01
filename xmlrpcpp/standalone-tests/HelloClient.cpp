// HelloClient.cpp : A simple xmlrpc client. Usage: HelloClient serverHost serverPort
// Link against xmlrpc lib and whatever socket libs your system needs (ws2_32.lib 
// on windows)
#include "xmlrpcpp/XmlRpc.h"
#include <iostream>
using namespace XmlRpc;

int main(int argc, char* argv[])
{
  if (argc != 3) {
    std::cerr << "Usage: HelloClient serverHost serverPort\n";
    return -1;
  }
  int port = atoi(argv[2]);
  //XmlRpc::setVerbosity(5);

  // Use introspection API to look up the supported methods
  XmlRpcClient c(argv[1], port);
  XmlRpcValue noArgs, result;
  for (int i = 0; i < 2000; i++)
  {
    if (c.execute("system.listMethods", noArgs, result))
      std::cout << "\nMethods:\n " << result << "\n\n";
    else
      std::cout << "Error calling 'listMethods'\n\n";
  }

  // Use introspection API to get the help string for the Hello method
  XmlRpcValue oneArg;
  oneArg[0] = "Hello";
  if (c.execute("system.methodHelp", oneArg, result))
    std::cout << "Help for 'Hello' method: " << result << "\n\n";
  else
    std::cout << "Error calling 'methodHelp'\n\n";

  // Call the Hello method
  if (c.execute("Hello", noArgs, result))
    std::cout << result << "\n\n";
  else
    std::cout << "Error calling 'Hello'\n\n";

  // Call the HelloName method
  oneArg[0] = "Chris";
  if (c.execute("HelloName", oneArg, result))
    std::cout << result << "\n\n";
  else
    std::cout << "Error calling 'HelloName'\n\n";

  // Add up an array of numbers
  XmlRpcValue numbers;
  numbers[0] = 33.33;
  numbers[1] = 112.57;
  numbers[2] = 76.1;
  std::cout << "numbers.size() is " << numbers.size() << std::endl;
  if (c.execute("Sum", numbers, result))
    std::cout << "Sum = " << double(result) << "\n\n";
  else
    std::cout << "Error calling 'Sum'\n\n";

  // Test the "no such method" fault
  if (c.execute("NoSuchMethod", numbers, result))
    std::cout << "NoSuchMethod call: fault: " << c.isFault() << ", result = " << result << std::endl;
  else
    std::cout << "Error calling 'Sum'\n";

  // Test the multicall method. It accepts one arg, an array of structs
  XmlRpcValue multicall;
  multicall[0][0]["methodName"] = "Sum";
  multicall[0][0]["params"][0] = 5.0;
  multicall[0][0]["params"][1] = 9.0;

  multicall[0][1]["methodName"] = "NoSuchMethod";
  multicall[0][1]["params"][0] = "";

  multicall[0][2]["methodName"] = "Sum";
  // Missing params

  multicall[0][3]["methodName"] = "Sum";
  multicall[0][3]["params"][0] = 10.5;
  multicall[0][3]["params"][1] = 12.5;

  if (c.execute("system.multicall", multicall, result))
    std::cout << "\nmulticall  result = " << result << std::endl;
  else
    std::cout << "\nError calling 'system.multicall'\n";

  return 0;
}
