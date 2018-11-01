
#include "xmlrpcpp/XmlRpcClient.h"

#include "xmlrpcpp/XmlRpcSocket.h"
#include "xmlrpcpp/XmlRpcUtil.h"
#include "xmlrpcpp/XmlRpcValue.h"

#include <stdio.h>
#include <stdlib.h>
#ifndef _WINDOWS
	# include <strings.h>
#endif
#include <string.h>


using namespace XmlRpc;

// Static data
const char XmlRpcClient::REQUEST_BEGIN[] = 
  "<?xml version=\"1.0\"?>\r\n"
  "<methodCall><methodName>";
const char XmlRpcClient::REQUEST_END_METHODNAME[] = "</methodName>\r\n";
const char XmlRpcClient::PARAMS_TAG[] = "<params>";
const char XmlRpcClient::PARAMS_ETAG[] = "</params>";
const char XmlRpcClient::PARAM_TAG[] = "<param>";
const char XmlRpcClient::PARAM_ETAG[] =  "</param>";
const char XmlRpcClient::REQUEST_END[] = "</methodCall>\r\n";
const char XmlRpcClient::METHODRESPONSE_TAG[] = "<methodResponse>";
const char XmlRpcClient::FAULT_TAG[] = "<fault>";


const char * XmlRpcClient::connectionStateStr(ClientConnectionState state) {
  switch(state) {
    case NO_CONNECTION:
      return "NO_CONNECTION";
    case CONNECTING:
      return "CONNECTING";
    case WRITE_REQUEST:
      return "WRITE_REQUEST";
    case READ_HEADER:
      return "READ_HEADER";
    case READ_RESPONSE:
      return "READ_RESPONSE";
    case IDLE:
      return "IDLE";
    default:
      return "UNKNOWN";
  }
}

XmlRpcClient::XmlRpcClient(const char* host, int port, const char* uri/*=0*/)
  : _connectionState(NO_CONNECTION),
  _host(host),
  _port(port),
  _sendAttempts(0),
  _bytesWritten(0),
  _executing(false),
  _eof(false),
  _isFault(false),
  _contentLength(0)
{
  XmlRpcUtil::log(1, "XmlRpcClient new client: host %s, port %d.", host, port);

  if (uri)
    _uri = uri;
  else
    _uri = "/RPC2";

  // Default to keeping the connection open until an explicit close is done
  setKeepOpen();
}


XmlRpcClient::~XmlRpcClient()
{
  this->close();
}

// Close the owned fd
void
XmlRpcClient::close()
{
  XmlRpcUtil::log(4, "XmlRpcClient::close: fd %d.", getfd());
  _connectionState = NO_CONNECTION;
  _disp.exit();
  _disp.removeSource(this);
  XmlRpcSource::close();
}


// Clear the referenced flag even if exceptions or errors occur.
struct ClearFlagOnExit {
  ClearFlagOnExit(bool& flag) : _flag(flag) {}
  ~ClearFlagOnExit() { _flag = false; }
  bool& _flag;
};

// Execute the named procedure on the remote server.
// Params should be an array of the arguments for the method.
// Returns true if the request was sent and a result received (although the result
// might be a fault).
bool
XmlRpcClient::execute(const char* method, XmlRpcValue const& params, XmlRpcValue& result)
{
  XmlRpcUtil::log(1, "XmlRpcClient::execute: method %s (_connectionState %s).", method, connectionStateStr(_connectionState));

  // This is not a thread-safe operation, if you want to do multithreading, use separate
  // clients for each thread. If you want to protect yourself from multiple threads
  // accessing the same client, replace this code with a real mutex.
  if (_executing)
    return false;

  _executing = true;
  ClearFlagOnExit cf(_executing);

  _sendAttempts = 0;
  _isFault = false;

  if ( ! setupConnection())
    return false;

  if ( ! generateRequest(method, params))
    return false;

  result.clear();
  double msTime = -1.0;   // Process until exit is called
  _disp.work(msTime);

  if (_connectionState != IDLE || ! parseResponse(result))
    return false;

  XmlRpcUtil::log(1, "XmlRpcClient::execute: method %s completed.", method);
  _response = "";
  return true;
}

// Execute the named procedure on the remote server, non-blocking.
// Params should be an array of the arguments for the method.
// Returns true if the request was sent and a result received (although the result
// might be a fault).
bool
XmlRpcClient::executeNonBlock(const char* method, XmlRpcValue const& params)
{
  XmlRpcUtil::log(1, "XmlRpcClient::executeNonBlock: method %s (_connectionState %s).", method, connectionStateStr(_connectionState));

  // This is not a thread-safe operation, if you want to do multithreading, use separate
  // clients for each thread. If you want to protect yourself from multiple threads
  // accessing the same client, replace this code with a real mutex.
  if (_executing)
    return false;

  _executing = true;
  ClearFlagOnExit cf(_executing);

  _sendAttempts = 0;
  _isFault = false;

  if ( ! setupConnection())
    return false;

  if ( ! generateRequest(method, params))
    return false;

  return true;
}

bool
XmlRpcClient::executeCheckDone(XmlRpcValue& result)
{
  result.clear();
  // Are we done yet?
  // If we lost connection, the call failed.
  if (_connectionState == NO_CONNECTION) {
    return true;
  }

  // Otherwise, assume the call is still in progress.
  if (_connectionState != IDLE) {
    return false;
  }

  if (! parseResponse(result))
  {
    // Hopefully the caller can determine that parsing failed.
  }
  //XmlRpcUtil::log(1, "XmlRpcClient::execute: method %s completed.", method);
  _response = "";
  return true;
}

// XmlRpcSource interface implementation
// Handle server responses. Called by the event dispatcher during execute.
unsigned
XmlRpcClient::handleEvent(unsigned eventType)
{
  if (eventType == XmlRpcDispatch::Exception)
  {
    if (_connectionState == WRITE_REQUEST && _bytesWritten == 0)
      XmlRpcUtil::error("Error in XmlRpcClient::handleEvent: could not connect to server (%s).", 
                       XmlRpcSocket::getErrorMsg().c_str());
    else
      XmlRpcUtil::error("Error in XmlRpcClient::handleEvent (state %s): %s.", 
                        connectionStateStr(_connectionState),
                        XmlRpcSocket::getErrorMsg().c_str());
    return 0;
  }

  if (_connectionState == WRITE_REQUEST)
    if ( ! writeRequest()) return 0;

  if (_connectionState == READ_HEADER)
    if ( ! readHeader()) return 0;

  if (_connectionState == READ_RESPONSE)
    if ( ! readResponse()) return 0;

  // This should probably always ask for Exception events too
  return (_connectionState == WRITE_REQUEST) 
        ? XmlRpcDispatch::WritableEvent : XmlRpcDispatch::ReadableEvent;
}


// Create the socket connection to the server if necessary
bool
XmlRpcClient::setupConnection()
{
  // If an error occurred last time through, or if the server closed the connection, close our end
  if ((_connectionState != NO_CONNECTION && _connectionState != IDLE) || _eof)
    close();

  _eof = false;
  if (_connectionState == NO_CONNECTION)
    if (! doConnect()) 
      return false;

  // Prepare to write the request
  _connectionState = WRITE_REQUEST;
  _bytesWritten = 0;

  // Notify the dispatcher to listen on this source (calls handleEvent when the socket is writable)
  _disp.removeSource(this);       // Make sure nothing is left over
  _disp.addSource(this, XmlRpcDispatch::WritableEvent | XmlRpcDispatch::Exception);

  return true;
}


// Connect to the xmlrpc server
bool
XmlRpcClient::doConnect()
{
  int fd = XmlRpcSocket::socket();
  if (fd < 0)
  {
    XmlRpcUtil::error("Error in XmlRpcClient::doConnect: Could not create socket (%s).", XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  XmlRpcUtil::log(3, "XmlRpcClient::doConnect: fd %d.", fd);
  this->setfd(fd);

  // Don't block on connect/reads/writes
  if ( ! XmlRpcSocket::setNonBlocking(fd))
  {
    this->close();
    XmlRpcUtil::error("Error in XmlRpcClient::doConnect: Could not set socket to non-blocking IO mode (%s).", XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  if ( ! XmlRpcSocket::connect(fd, _host, _port))
  {
    this->close();
    XmlRpcUtil::error("Error in XmlRpcClient::doConnect: Could not connect to server (%s).", XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  return true;
}

// Encode the request to call the specified method with the specified parameters into xml
bool
XmlRpcClient::generateRequest(const char* methodName, XmlRpcValue const& params)
{
  std::string body = REQUEST_BEGIN;
  body += methodName;
  body += REQUEST_END_METHODNAME;

  // If params is an array, each element is a separate parameter
  if (params.valid()) {
    body += PARAMS_TAG;
    if (params.getType() == XmlRpcValue::TypeArray)
    {
      for (int i=0; i<params.size(); ++i) {
        body += PARAM_TAG;
        body += params[i].toXml();
        body += PARAM_ETAG;
      }
    }
    else
    {
      body += PARAM_TAG;
      body += params.toXml();
      body += PARAM_ETAG;
    }
      
    body += PARAMS_ETAG;
  }
  body += REQUEST_END;

  std::string header = generateHeader(body.length());
  XmlRpcUtil::log(4, "XmlRpcClient::generateRequest: header is %d bytes, content-length is %d.", 
                  header.length(), body.length());

  _request = header + body;
  return true;
}

// Prepend http headers
std::string
XmlRpcClient::generateHeader(size_t length) const
{
  std::string header = 
    "POST " + _uri + " HTTP/1.1\r\n"
    "User-Agent: ";
  header += XMLRPC_VERSION;
  header += "\r\nHost: ";
  header += _host;

  char buff[40];
  snprintf(buff,40,":%d\r\n", _port);

  header += buff;
  header += "Content-Type: text/xml\r\nContent-length: ";

  snprintf(buff,40,"%zu\r\n\r\n", length);

  return header + buff;
}

bool
XmlRpcClient::writeRequest()
{
  if (_bytesWritten == 0)
    XmlRpcUtil::log(5, "XmlRpcClient::writeRequest (attempt %d):\n%s\n", _sendAttempts+1, _request.c_str());

  // Try to write the request
  if ( ! XmlRpcSocket::nbWrite(this->getfd(), _request, &_bytesWritten)) {
    XmlRpcUtil::error("Error in XmlRpcClient::writeRequest: write error (%s).",XmlRpcSocket::getErrorMsg().c_str());
    // If the write fails, we had an unrecoverable error. Close the socket.
    close();
    return false;
  }
    
  XmlRpcUtil::log(3, "XmlRpcClient::writeRequest: wrote %d of %d bytes.", _bytesWritten, _request.length());

  // Wait for the result
  if (_bytesWritten == int(_request.length())) {
    _header = "";
    _response = "";
    _connectionState = READ_HEADER;
  } else {
    // On partial write, remove the portion of the output that was written from
    // the request buffer.
    _request = _request.substr(_bytesWritten);
    _bytesWritten = 0;
  }
  return true;
}


// Read the header from the response
bool
XmlRpcClient::readHeader()
{
  // Read available data
  if ( ! XmlRpcSocket::nbRead(this->getfd(), _header, &_eof) ||
       (_eof && _header.length() == 0)) {

    // If we haven't read any data yet and this is a keep-alive connection, the server may
    // have timed out, so we try one more time.
    if (getKeepOpen() && _header.length() == 0 && _sendAttempts++ == 0) {
      XmlRpcUtil::log(4, "XmlRpcClient::readHeader: re-trying connection");
      XmlRpcSource::close();
      _connectionState = NO_CONNECTION;
      _eof = false;
      return setupConnection();
    }

    XmlRpcUtil::error("Error in XmlRpcClient::readHeader: error while reading "
                      "header (%s) on fd %d.",
                      XmlRpcSocket::getErrorMsg().c_str(), getfd());
    // Read failed; this means the socket is in an unrecoverable state.
    // Close the socket.
    close();
    return false;
  }

  XmlRpcUtil::log(4, "XmlRpcClient::readHeader: client has read %d bytes", _header.length());

  char *hp = (char*)_header.c_str();  // Start of header
  char *ep = hp + _header.length();   // End of string
  char *bp = 0;                       // Start of body
  char *lp = 0;                       // Start of content-length value

  for (char *cp = hp; (bp == 0) && (cp < ep); ++cp) {
    if ((ep - cp > 16) && (strncasecmp(cp, "Content-length: ", 16) == 0))
      lp = cp + 16;
    else if ((ep - cp > 4) && (strncmp(cp, "\r\n\r\n", 4) == 0))
      bp = cp + 4;
    else if ((ep - cp > 2) && (strncmp(cp, "\n\n", 2) == 0))
      bp = cp + 2;
  }

  // If we haven't gotten the entire header yet, return (keep reading)
  if (bp == 0) {
    if (_eof)          // EOF in the middle of a response is an error
    {
      XmlRpcUtil::error("Error in XmlRpcClient::readHeader: EOF while reading header");
      close();
      return false;   // Close the connection
    }
    
    return true;  // Keep reading
  }

  // Decode content length
  if (lp == 0) {
    XmlRpcUtil::error("Error XmlRpcClient::readHeader: No Content-length specified");
    // Close the socket because we can't make further use of it.
    close();
    return false;   // We could try to figure it out by parsing as we read, but for now...
  }

  _contentLength = atoi(lp);
  if (_contentLength <= 0) {
    XmlRpcUtil::error("Error in XmlRpcClient::readHeader: Invalid Content-length specified (%d).", _contentLength);
    // Close the socket because we can't make further use of it.
    close();
    return false;
  }
  	
  XmlRpcUtil::log(4, "client read content length: %d", _contentLength);

  // Otherwise copy non-header data to response buffer and set state to read response.
  _response = bp;
  _header = "";   // should parse out any interesting bits from the header (connection, etc)...
  _connectionState = READ_RESPONSE;
  return true;    // Continue monitoring this source
}

    
bool
XmlRpcClient::readResponse()
{
  // If we dont have the entire response yet, read available data
  if (int(_response.length()) < _contentLength) {
    std::string buff;
    if ( ! XmlRpcSocket::nbRead(this->getfd(), buff, &_eof)) {
      XmlRpcUtil::error("Error in XmlRpcClient::readResponse: read error (%s).",XmlRpcSocket::getErrorMsg().c_str());
      // nbRead returned an error, indicating that the socket is in a bad state.
      // close it and stop monitoring this client.
      close();
      return false;
    }
    _response += buff;

    // If we haven't gotten the entire _response yet, return (keep reading)
    if (int(_response.length()) < _contentLength) {
      if (_eof) {
        XmlRpcUtil::error("Error in XmlRpcClient::readResponse: EOF while reading response");
        // nbRead returned an eof, indicating that the socket is disconnected.
        // close it and stop monitoring this client.
        close();
        return false;
      }
      return true;
    }
  }

  // Otherwise, parse and return the result
  XmlRpcUtil::log(3, "XmlRpcClient::readResponse (read %d bytes)", _response.length());
  XmlRpcUtil::log(5, "response:\n%s", _response.c_str());

  _connectionState = IDLE;

  return false;    // Stop monitoring this source (causes return from work)
}


// Convert the response xml into a result value
bool
XmlRpcClient::parseResponse(XmlRpcValue& result)
{
  // Parse response xml into result
  int offset = 0;
  if ( ! XmlRpcUtil::findTag(METHODRESPONSE_TAG,_response,&offset)) {
    XmlRpcUtil::error("Error in XmlRpcClient::parseResponse: Invalid response - no methodResponse. Response:\n%s", _response.c_str());
    return false;
  }

  // Expect either <params><param>... or <fault>...
  if ((XmlRpcUtil::nextTagIs(PARAMS_TAG,_response,&offset) &&
       XmlRpcUtil::nextTagIs(PARAM_TAG,_response,&offset)) ||
      (XmlRpcUtil::nextTagIs(FAULT_TAG,_response,&offset) && (_isFault = true)))
  {
    if ( ! result.fromXml(_response, &offset)) {
      XmlRpcUtil::error("Error in XmlRpcClient::parseResponse: Invalid response value. Response:\n%s", _response.c_str());
      _response = "";
      return false;
    }
  } else {
    XmlRpcUtil::error("Error in XmlRpcClient::parseResponse: Invalid response - no param or fault tag. Response:\n%s", _response.c_str());
    _response = "";
    return false;
  }
      
  _response = "";
  return result.valid();
}

