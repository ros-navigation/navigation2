#ifndef _XMLRPCUTIL_H_
#define _XMLRPCUTIL_H_
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

#if defined(_MSC_VER)
# define snprintf	    _snprintf_s
# define vsnprintf    _vsnprintf_s
# define strcasecmp	  _stricmp
# define strncasecmp	_strnicmp
#elif defined(__BORLANDC__)
# define strcasecmp stricmp
# define strncasecmp strnicmp
#endif

namespace XmlRpc {

  //! An interface allowing custom handling of error message reporting.
  class XmlRpcErrorHandler {
  public:
    virtual ~XmlRpcErrorHandler() { }

    //! Returns a pointer to the currently installed error handling object.
    static XmlRpcErrorHandler* getErrorHandler()
    { return _errorHandler; }

    //! Specifies the error handler.
    static void setErrorHandler(XmlRpcErrorHandler* eh)
    { _errorHandler = eh; }

    //! Report an error. Custom error handlers should define this method.
    virtual void error(const char* msg) = 0;

  protected:
    static XMLRPCPP_DECL XmlRpcErrorHandler* _errorHandler;
  };

  //! An interface allowing custom handling of informational message reporting.
  class XmlRpcLogHandler {
  public:
    virtual ~XmlRpcLogHandler() { }

    //! Returns a pointer to the currently installed message reporting object.
    static XmlRpcLogHandler* getLogHandler()
    { return _logHandler; }

    //! Specifies the message handler.
    static void setLogHandler(XmlRpcLogHandler* lh)
    { _logHandler = lh; }

    //! Returns the level of verbosity of informational messages. 0 is no output, 5 is very verbose.
    static int getVerbosity()
    { return _verbosity; }

    //! Specify the level of verbosity of informational messages. 0 is no output, 5 is very verbose.
    static void setVerbosity(int v)
    { _verbosity = v; }

    //! Output a message. Custom error handlers should define this method.
    virtual void log(int level, const char* msg) = 0;

  protected:
    static XMLRPCPP_DECL XmlRpcLogHandler* _logHandler;
    static XMLRPCPP_DECL int _verbosity;
  };

  //! Returns log message verbosity. This is short for XmlRpcLogHandler::getVerbosity()
  int getVerbosity();
  //! Sets log message verbosity. This is short for XmlRpcLogHandler::setVerbosity(level)
  void setVerbosity(int level);

  //! Version identifier
  extern const char XMLRPC_VERSION[];

  //! Utilities for XML parsing, encoding, and decoding and message handlers.
  class XMLRPCPP_DECL XmlRpcUtil {
  public:
    // hokey xml parsing
    //! Returns contents between <tag> and </tag>, updates offset to char after </tag>
    static std::string parseTag(const char* tag, std::string const& xml, int* offset);

    //! Returns true if the tag is found and updates offset to the char after the tag
    static bool findTag(const char* tag, std::string const& xml, int* offset);

    //! Returns the next tag and updates offset to the char after the tag, or empty string
    //! if the next non-whitespace character is not '<'
    static std::string getNextTag(std::string const& xml, int* offset);

    //! Returns true if the tag is found at the specified offset (modulo any whitespace)
    //! and updates offset to the char after the tag
    static bool nextTagIs(const char* tag, std::string const& xml, int* offset);


    //! Convert raw text to encoded xml.
    static std::string xmlEncode(const std::string& raw);

    //! Convert encoded xml to raw text
    static std::string xmlDecode(const std::string& encoded);


    //! Dump messages somewhere
    static void log(int level, const char* fmt, ...);

    //! Dump error messages somewhere
    static void error(const char* fmt, ...);

  };
} // namespace XmlRpc

#endif // _XMLRPCUTIL_H_
