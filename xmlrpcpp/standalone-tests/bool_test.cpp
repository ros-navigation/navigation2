// bool_test.cpp : make sure bool variables copy around ok
#include "xmlrpcpp/XmlRpc.h"
#include <iostream>
#include <stdlib.h>

using namespace XmlRpc;
using namespace std;

int main(int argc, char* argv[])
{
  XmlRpcValue v(bool(false));
  cout << v.toXml() << endl;
  XmlRpcValue v2;
  v2[0] = int(1);
  v2[1] = string();
  v2[2] = XmlRpcValue(false);
  cout << v2.toXml() << endl;

  return 0;
}

