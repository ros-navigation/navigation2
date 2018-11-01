
#ifndef _XMLRPCEXCEPTION_H_
#define _XMLRPCEXCEPTION_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#ifndef MAKEDEPEND
# include <string>
#endif

#include "xmlrpcpp/XmlRpcDecl.h"


namespace XmlRpc {

  //! A class representing an error.
  //! If server methods throw this exception, a fault response is returned
  //! to the client.
  class XMLRPCPP_DECL XmlRpcException {
  public:
    //! Constructor
    //!   @param message  A descriptive error message
    //!   @param code     An integer error code
    XmlRpcException(const std::string& message, int code=-1) :
        _message(message), _code(code) {}

    //! Return the error message.
    const std::string& getMessage() const { return _message; }

    //! Return the error code.
    int getCode() const { return _code; }

  private:
    std::string _message;
    int _code;
  };

}

#endif	// _XMLRPCEXCEPTION_H_
