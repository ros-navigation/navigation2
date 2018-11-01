
#ifndef _XMLRPCSOURCE_H_
#define _XMLRPCSOURCE_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#include "xmlrpcpp/XmlRpcDecl.h"

namespace XmlRpc {

  //! An RPC source represents a file descriptor to monitor
  class XMLRPCPP_DECL XmlRpcSource {
  public:
    //! Constructor
    //!  @param fd The socket file descriptor to monitor.
    //!  @param deleteOnClose If true, the object deletes itself when close is called.
    XmlRpcSource(int fd = -1, bool deleteOnClose = false);

    //! Destructor
    virtual ~XmlRpcSource();

    //! Return the file descriptor being monitored.
    int getfd() const { return _fd; }
    //! Specify the file descriptor to monitor.
    void setfd(int fd) { _fd = fd; }

    //! Return whether the file descriptor should be kept open if it is no longer monitored.
    bool getKeepOpen() const { return _keepOpen; }
    //! Specify whether the file descriptor should be kept open if it is no longer monitored.
    void setKeepOpen(bool b=true) { _keepOpen = b; }

    //! Close the owned fd. If deleteOnClose was specified at construction, the object is deleted.
    virtual void close();

    //! Return true to continue monitoring this source
    virtual unsigned handleEvent(unsigned eventType) = 0;

  private:

    // Socket. This should really be a SOCKET (an alias for unsigned int*) on windows...
    int _fd;

    // In the server, a new source (XmlRpcServerConnection) is created
    // for each connected client. When each connection is closed, the
    // corresponding source object is deleted.
    bool _deleteOnClose;

    // In the client, keep connections open if you intend to make multiple calls.
    bool _keepOpen;
  };
} // namespace XmlRpc

#endif //_XMLRPCSOURCE_H_
