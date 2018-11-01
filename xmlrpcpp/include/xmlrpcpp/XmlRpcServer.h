// this file modified by Morgan Quigley on 22 Apr 2008.
// added features: server can be opened on port 0 and you can read back
// what port the OS gave you

#ifndef _XMLRPCSERVER_H_
#define _XMLRPCSERVER_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#ifndef MAKEDEPEND
# include <map>
# include <string>
# include <vector>
# if defined(_WINDOWS)
#  include <winsock2.h>
# else
#  include <poll.h>
# endif
#endif

#include "xmlrpcpp/XmlRpcDispatch.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcDecl.h"

namespace XmlRpc {


  // An abstract class supporting XML RPC methods
  class XmlRpcServerMethod;

  // Class representing connections to specific clients
  class XmlRpcServerConnection;

  // Class representing argument and result values
  class XmlRpcValue;


  //! A class to handle XML RPC requests
  class XMLRPCPP_DECL XmlRpcServer : public XmlRpcSource {
  public:
    //! Create a server object.
    XmlRpcServer();
    //! Destructor.
    virtual ~XmlRpcServer();

    //! Specify whether introspection is enabled or not. Default is not enabled.
    void enableIntrospection(bool enabled=true);

    //! Add a command to the RPC server
    void addMethod(XmlRpcServerMethod* method);

    //! Remove a command from the RPC server
    void removeMethod(XmlRpcServerMethod* method);

    //! Remove a command from the RPC server by name
    void removeMethod(const std::string& methodName);

    //! Look up a method by name
    XmlRpcServerMethod* findMethod(const std::string& name) const;

    //! Create a socket, bind to the specified port, and
    //! set it in listen mode to make it available for clients.
    bool bindAndListen(int port, int backlog = 5);

    //! Process client requests for the specified time
    void work(double msTime);

    //! Temporarily stop processing client requests and exit the work() method.
    void exit();

    //! Close all connections with clients and the socket file descriptor
    void shutdown();

    //! Introspection support
    void listMethods(XmlRpcValue& result);

    // XmlRpcSource interface implementation

    //! Handle client connection requests
    virtual unsigned handleEvent(unsigned eventType);

    //! Remove a connection from the dispatcher
    virtual void removeConnection(XmlRpcServerConnection*);

    inline int get_port() { return _port; }

    XmlRpcDispatch *get_dispatch() { return &_disp; }

  protected:

    //! Accept a client connection request
    virtual unsigned acceptConnection();

    //! Create a new connection object for processing requests from a specific client.
    virtual XmlRpcServerConnection* createConnection(int socket);

    //! Count number of free file descriptors
    int countFreeFDs();

    // Whether the introspection API is supported by this server
    bool _introspectionEnabled;

    // Event dispatcher
    XmlRpcDispatch _disp;

    // Collection of methods. This could be a set keyed on method name if we wanted...
    typedef std::map< std::string, XmlRpcServerMethod* > MethodMap;
    MethodMap _methods;

    // system methods
    XmlRpcServerMethod* _listMethods;
    XmlRpcServerMethod* _methodHelp;

    int _port;

    // Flag indicating that accept had an error and needs to be retried.
    bool _accept_error;
    // If we cannot accept(), retry after this many seconds. Hopefully there
    // will be more free file descriptors later.
    static const double ACCEPT_RETRY_INTERVAL_SEC;
    // Retry time for accept.
    double _accept_retry_time_sec;

    // Minimum number of free file descriptors before rejecting clients.
    static const int FREE_FD_BUFFER;
    // List of all file descriptors, used for counting open files.
    std::vector<struct pollfd> pollfds;
  };
} // namespace XmlRpc

#endif //_XMLRPCSERVER_H_
