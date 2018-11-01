// this file modified by Morgan Quigley on 22 Apr 2008.
// added features: server can be opened on port 0 and you can read back
// what port the OS gave you

#include "xmlrpcpp/XmlRpcSocket.h"
#include "xmlrpcpp/XmlRpcUtil.h"

#ifndef MAKEDEPEND

#if defined(_WINDOWS)
# include <stdio.h>
# include <winsock2.h>
# include <ws2tcpip.h>
//# pragma lib(WS2_32.lib)

// MS VS10 actually has these definitions (as opposed to earlier versions),
// so if present, temporarily disable them and reset to WSA codes for this file only.
#ifdef EAGAIN
  #undef EAGAIN
#endif
#ifdef EINTR
  #undef EINTR
#endif
#ifdef EINPROGRESS
  #undef EINPROGRESS
#endif
#ifdef  EWOULDBLOCK
  #undef EWOULDBLOCK
#endif
#ifdef ETIMEDOUT
  #undef ETIMEDOUT
#endif
# define EAGAIN		WSATRY_AGAIN
# define EINTR			WSAEINTR
# define EINPROGRESS	WSAEINPROGRESS
# define EWOULDBLOCK	WSAEWOULDBLOCK
# define ETIMEDOUT	    WSAETIMEDOUT
#else
extern "C" {
# include <unistd.h>
# include <stdio.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <netdb.h>
# include <errno.h>
# include <fcntl.h>
# include <string.h>
# include <stdlib.h>
# include <arpa/inet.h>
}
#endif  // _WINDOWS

#endif // MAKEDEPEND

// MSG_NOSIGNAL does not exists on OS X
#if defined(__APPLE__) || defined(__MACH__)
# ifndef MSG_NOSIGNAL
#   define MSG_NOSIGNAL SO_NOSIGPIPE
# endif
#endif


using namespace XmlRpc;


bool XmlRpcSocket::s_use_ipv6_ = false;

#if defined(_WINDOWS)

static void initWinSock()
{
  static bool wsInit = false;
  if (! wsInit)
  {
    WORD wVersionRequested = MAKEWORD( 2, 0 );
    WSADATA wsaData;
    WSAStartup(wVersionRequested, &wsaData);
    wsInit = true;
  }
}

#else

#define initWinSock()

#endif // _WINDOWS


// These errors are not considered fatal for an IO operation; the operation will be re-tried.
static inline bool
nonFatalError()
{
  int err = XmlRpcSocket::getError();
  return (err == EINPROGRESS || err == EAGAIN || err == EWOULDBLOCK || err == EINTR);
}

int
XmlRpcSocket::socket()
{
  initWinSock();
  return (int) ::socket(s_use_ipv6_ ? AF_INET6 : AF_INET, SOCK_STREAM, 0);
}


void
XmlRpcSocket::close(int fd)
{
  XmlRpcUtil::log(4, "XmlRpcSocket::close: fd %d.", fd);
#if defined(_WINDOWS)
  closesocket(fd);
#else
  ::close(fd);
#endif // _WINDOWS
}




bool
XmlRpcSocket::setNonBlocking(int fd)
{
#if defined(_WINDOWS)
  unsigned long flag = 1;
  return (ioctlsocket((SOCKET)fd, FIONBIO, &flag) == 0);
#else
  return (fcntl(fd, F_SETFL, O_NONBLOCK) == 0);
#endif // _WINDOWS
}


bool
XmlRpcSocket::setReuseAddr(int fd)
{
  // Allow this port to be re-bound immediately so server re-starts are not delayed
  int sflag = 1;
  return (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&sflag, sizeof(sflag)) == 0);
}


// Bind to a specified port
bool
XmlRpcSocket::bind(int fd, int port)
{
  sockaddr_storage ss;
  socklen_t ss_len;
  memset(&ss, 0, sizeof(ss));

  if (s_use_ipv6_)
  {
    sockaddr_in6 *address = (sockaddr_in6 *)&ss;
    ss_len = sizeof(sockaddr_in6);

    address->sin6_family = AF_INET6;
    address->sin6_addr = in6addr_any;
    address->sin6_port = htons((u_short) port);
  }
  else
  {
    sockaddr_in *address = (sockaddr_in *)&ss;
    ss_len = sizeof(sockaddr_in);

    address->sin_family = AF_INET;
    address->sin_addr.s_addr = htonl(INADDR_ANY);
    address->sin_port = htons((u_short) port);
  }

  return (::bind(fd, (sockaddr*)&ss, ss_len) == 0);
}


// Set socket in listen mode
bool
XmlRpcSocket::listen(int fd, int backlog)
{
  return (::listen(fd, backlog) == 0);
}


int
XmlRpcSocket::accept(int fd)
{
  struct sockaddr_in addr;
#if defined(_WINDOWS)
  int
#else
  socklen_t
#endif
    addrlen = sizeof(addr);
  // accept will truncate the address if the buffer is too small.
  // As we are not using it, no special case for IPv6
  // has to be made.
  return (int) ::accept(fd, (struct sockaddr*)&addr, &addrlen);
}



// Connect a socket to a server (from a client)
bool
XmlRpcSocket::connect(int fd, const std::string& host, int port)
{
  sockaddr_storage ss;
  socklen_t ss_len;
  memset(&ss, 0, sizeof(ss));

  struct addrinfo* addr;
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  int getaddr_err = getaddrinfo(host.c_str(), NULL, &hints, &addr);
  if (0 != getaddr_err) {
#if !defined(_WINDOWS)
    if(getaddr_err == EAI_SYSTEM) {
      XmlRpcUtil::error("Couldn't find an %s address for [%s]: %s\n", s_use_ipv6_ ? "AF_INET6" : "AF_INET", host.c_str(), XmlRpcSocket::getErrorMsg().c_str());
    } else {
#else
    {
#endif
      XmlRpcUtil::error("Couldn't find an %s address for [%s]: %s\n", s_use_ipv6_ ? "AF_INET6" : "AF_INET", host.c_str(), gai_strerror(getaddr_err));
    }
    return false;
  }

  bool found = false;
  struct addrinfo* it = addr;

  for (; it; it = it->ai_next)
  {
    if (!s_use_ipv6_ && it->ai_family == AF_INET)
    {
      sockaddr_in *address = (sockaddr_in *)&ss;
      ss_len = sizeof(sockaddr_in);

      memcpy(address, it->ai_addr, it->ai_addrlen);
      address->sin_family = it->ai_family;
      address->sin_port = htons((u_short) port);

      XmlRpcUtil::log(5, "found host as %s\n", inet_ntoa(address->sin_addr));
      found = true;
      break;
    }
    if (s_use_ipv6_ && it->ai_family == AF_INET6)
    {
      sockaddr_in6 *address = (sockaddr_in6 *)&ss;
      ss_len = sizeof(sockaddr_in6);

      memcpy(address, it->ai_addr, it->ai_addrlen);
      address->sin6_family = it->ai_family;
      address->sin6_port = htons((u_short) port);

      char buf[128];
      // TODO IPV6: check if this also works under Windows
      XmlRpcUtil::log(5, "found ipv6 host as %s\n", inet_ntop(AF_INET6, (void*)&(address->sin6_addr), buf, sizeof(buf)));
      found = true;
      break;
    }

  }

  if (!found)
  {
    XmlRpcUtil::error("Couldn't find an %s address for [%s]\n", s_use_ipv6_ ? "AF_INET6" : "AF_INET", host.c_str());
    freeaddrinfo(addr);
    return false;
  }

  // For asynch operation, this will return EWOULDBLOCK (windows) or
  // EINPROGRESS (linux) and we just need to wait for the socket to be writable...
  int result = ::connect(fd, (sockaddr*)&ss, ss_len);
  bool success = true;
  if (result != 0 ) {
    int error = getError();
    // platform check here, EWOULDBLOCK on WIN32 and EINPROGRESS otherwise
#if defined(_WINDOWS)
    if (error != EWOULDBLOCK) {
#else
    if (error != EINPROGRESS) {
#endif
      XmlRpcUtil::error("::connect error = %s\n", getErrorMsg(error).c_str());
      success = false;
    }
  }

  freeaddrinfo(addr);

  return success;
}



// Read available text from the specified socket. Returns false on error.
bool
XmlRpcSocket::nbRead(int fd, std::string& s, bool *eof)
{
  const int READ_SIZE = 4096;   // Number of bytes to attempt to read at a time
  char readBuf[READ_SIZE];

  bool wouldBlock = false;
  *eof = false;

  while ( ! wouldBlock && ! *eof) {
#if defined(_WINDOWS)
    int n = recv(fd, readBuf, READ_SIZE-1, 0);
#else
    int n = read(fd, readBuf, READ_SIZE-1);
#endif
    XmlRpcUtil::log(5, "XmlRpcSocket::nbRead: read/recv returned %d.", n);

    if (n > 0) {
      readBuf[n] = 0;
      s.append(readBuf, n);
    } else if (n == 0) {
      *eof = true;
    } else if (nonFatalError()) {
      wouldBlock = true;
    } else {
      return false;   // Error
    }
  }
  return true;
}


// Write text to the specified socket. Returns false on error.
bool
XmlRpcSocket::nbWrite(int fd, const std::string& s, int *bytesSoFar)
{
  int nToWrite = int(s.length()) - *bytesSoFar;
  char *sp = const_cast<char*>(s.c_str()) + *bytesSoFar;
  bool wouldBlock = false;

  while ( nToWrite > 0 && ! wouldBlock ) {
#if defined(_WINDOWS)
    int n = send(fd, sp, nToWrite, 0);
#else
    int n = write(fd, sp, nToWrite);
#endif
    XmlRpcUtil::log(5, "XmlRpcSocket::nbWrite: send/write returned %d.", n);

    if (n > 0) {
      sp += n;
      *bytesSoFar += n;
      nToWrite -= n;
    } else if (nonFatalError()) {
      wouldBlock = true;
    } else {
      return false;   // Error
    }
  }
  return true;
}


// Returns last errno
int
XmlRpcSocket::getError()
{
#if defined(_WINDOWS)
  return WSAGetLastError();
#else
  return errno;
#endif
}


// Returns message corresponding to last errno
std::string
XmlRpcSocket::getErrorMsg()
{
  return getErrorMsg(getError());
}

// Returns message corresponding to errno... well, it should anyway
std::string
XmlRpcSocket::getErrorMsg(int error)
{
  char err[60];
#ifdef _MSC_VER
  strerror_s(err,60,error);
#else
  snprintf(err,sizeof(err),"%s",strerror(error));
#endif
  return std::string(err);
}

int XmlRpcSocket::get_port(int socket)
{
  sockaddr_storage ss;
  socklen_t ss_len = sizeof(ss);
  if(getsockname(socket, (sockaddr *)&ss, &ss_len) == 0) {
    sockaddr_in *sin = (sockaddr_in *)&ss;
    sockaddr_in6 *sin6 = (sockaddr_in6 *)&ss;

    switch (ss.ss_family)
    {
      case AF_INET:
        return ntohs(sin->sin_port);
      case AF_INET6:
        return ntohs(sin6->sin6_port);
    }
  }
  return 0;
}

