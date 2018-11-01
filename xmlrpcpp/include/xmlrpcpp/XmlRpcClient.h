
#ifndef _XMLRPCCLIENT_H_
#define _XMLRPCCLIENT_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif


#ifndef MAKEDEPEND
# include <string>
#endif

#include "xmlrpcpp/XmlRpcDispatch.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcDecl.h"

namespace XmlRpc {

  // Arguments and results are represented by XmlRpcValues
  class XmlRpcValue;

  //! A class to send XML RPC requests to a server and return the results.
  class XMLRPCPP_DECL XmlRpcClient : public XmlRpcSource {
  public:
    // Static data
    static const char REQUEST_BEGIN[];
    static const char REQUEST_END_METHODNAME[];
    static const char PARAMS_TAG[];
    static const char PARAMS_ETAG[];
    static const char PARAM_TAG[];
    static const char PARAM_ETAG[];
    static const char REQUEST_END[];
    // Result tags
    static const char METHODRESPONSE_TAG[];
    static const char FAULT_TAG[];

    //! Construct a client to connect to the server at the specified host:port address
    //!  @param host The name of the remote machine hosting the server
    //!  @param port The port on the remote machine where the server is listening
    //!  @param uri  An optional string to be sent as the URI in the HTTP GET header
    XmlRpcClient(const char* host, int port, const char* uri=0);

    //! Destructor
    virtual ~XmlRpcClient();

    //! Execute the named procedure on the remote server.
    //!  @param method The name of the remote procedure to execute
    //!  @param params An array of the arguments for the method
    //!  @param result The result value to be returned to the client
    //!  @return true if the request was sent and a result received 
    //!   (although the result might be a fault).
    //!
    //! Currently this is a synchronous (blocking) implementation (execute
    //! does not return until it receives a response or an error). Use isFault()
    //! to determine whether the result is a fault response.
    bool execute(const char* method, XmlRpcValue const& params, XmlRpcValue& result);

    bool executeNonBlock(const char* method, XmlRpcValue const& params);
    bool executeCheckDone(XmlRpcValue& result);

    //! Returns true if the result of the last execute() was a fault response.
    bool isFault() const { return _isFault; }


    // XmlRpcSource interface implementation
    //! Close the connection
    virtual void close();

    //! Handle server responses. Called by the event dispatcher during execute.
    //!  @param eventType The type of event that occurred. 
    //!  @see XmlRpcDispatch::EventType
    virtual unsigned handleEvent(unsigned eventType);

  protected:
    // Execution processing helpers
    virtual bool doConnect();
    virtual bool setupConnection();

    virtual bool generateRequest(const char* method, XmlRpcValue const& params);
    virtual std::string generateHeader(size_t length) const;
    virtual bool writeRequest();
    virtual bool readHeader();
    virtual bool readResponse();
    virtual bool parseResponse(XmlRpcValue& result);

    // Possible IO states for the connection
    enum ClientConnectionState { NO_CONNECTION, CONNECTING, WRITE_REQUEST, READ_HEADER, READ_RESPONSE, IDLE };
    ClientConnectionState _connectionState;

    static const char * connectionStateStr(ClientConnectionState state);

    // Server location
    std::string _host;
    std::string _uri;
    int _port;
  public:
    const std::string &getHost() { return _host; }
    const std::string &getUri()  { return _uri; }
    int getPort() const { return _port; }
    
    // The xml-encoded request, http header of response, and response xml
    std::string _request;
    std::string _header;
    std::string _response;

    // Number of times the client has attempted to send the request
    int _sendAttempts;

    // NOTE(austin): Having multiple variables that imply that the fd is valid
    //               smells funny.
    // Number of bytes of the request that have been written to the socket so far
    int _bytesWritten;

    // True if we are currently executing a request. If you want to multithread,
    // each thread should have its own client.
    bool _executing;

    // True if the server closed the connection
    bool _eof;

    // True if a fault response was returned by the server
    bool _isFault;

    // Number of bytes expected in the response body (parsed from response header)
    int _contentLength;

    // Event dispatcher
    XmlRpcDispatch _disp;

  };	// class XmlRpcClient

}	// namespace XmlRpc

#endif	// _XMLRPCCLIENT_H_
