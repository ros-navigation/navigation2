
#include "xmlrpcpp/XmlRpcDispatch.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcUtil.h"

#include "ros/time.h"

#include <math.h>
#include <errno.h>
#include <sys/timeb.h>

#if defined(_WINDOWS)
# include <winsock2.h>
static inline int poll( struct pollfd *pfd, int nfds, int timeout)
{
  return WSAPoll(pfd, nfds, timeout);
}

# define USE_FTIME
# if defined(_MSC_VER)
#  define timeb _timeb
#  define ftime _ftime_s
# endif
#else
# include <sys/poll.h>
# include <sys/time.h>
#endif  // _WINDOWS


using namespace XmlRpc;


XmlRpcDispatch::XmlRpcDispatch()
{
  _endTime = -1.0;
  _doClear = false;
  _inWork = false;
}


XmlRpcDispatch::~XmlRpcDispatch()
{
}

// Monitor this source for the specified events and call its event handler
// when the event occurs
void
XmlRpcDispatch::addSource(XmlRpcSource* source, unsigned mask)
{
  _sources.push_back(MonitoredSource(source, mask));
}

// Stop monitoring this source. Does not close the source.
void
XmlRpcDispatch::removeSource(XmlRpcSource* source)
{
  for (SourceList::iterator it=_sources.begin(); it!=_sources.end(); ++it)
    if (it->getSource() == source)
    {
      _sources.erase(it);
      break;
    }
}


// Modify the types of events to watch for on this source
void
XmlRpcDispatch::setSourceEvents(XmlRpcSource* source, unsigned eventMask)
{
  for (SourceList::iterator it=_sources.begin(); it!=_sources.end(); ++it)
    if (it->getSource() == source)
    {
      it->getMask() = eventMask;
      break;
    }
}



// Watch current set of sources and process events
void
XmlRpcDispatch::work(double timeout)
{
  // Loosely based on `man select` > Correspondence between select() and poll() notifications
  // and cloudius-systems/osv#35, cloudius-systems/osv@b53d39a using poll to emulate select
  const unsigned POLLIN_REQ = POLLIN; // Request read
  const unsigned POLLIN_CHK = (POLLIN | POLLHUP | POLLERR); // Readable or connection lost
  const unsigned POLLOUT_REQ = POLLOUT; // Request write
  const unsigned POLLOUT_CHK = (POLLOUT | POLLERR); // Writable or connection lost
#if !defined(_WINDOWS)
  const unsigned POLLEX_REQ = POLLPRI; // Out-of-band data received
  const unsigned POLLEX_CHK = (POLLPRI | POLLNVAL); // Out-of-band data or invalid fd
#else
  const unsigned POLLEX_REQ = POLLRDBAND; // Out-of-band data received
  const unsigned POLLEX_CHK = (POLLRDBAND | POLLNVAL); // Out-of-band data or invalid fd
#endif

  // Compute end time
  _endTime = (timeout < 0.0) ? -1.0 : (getTime() + timeout);
  _doClear = false;
  _inWork = true;
  int timeout_ms = static_cast<int>(floor(timeout * 1000.));

  // Only work while there is something to monitor
  while (_sources.size() > 0) {

    // Construct the sets of descriptors we are interested in
    const unsigned source_cnt = _sources.size();
    std::vector<pollfd> fds(source_cnt);
    std::vector<XmlRpcSource *> sources(source_cnt);

    SourceList::iterator it;
    std::size_t i = 0;
    for (it=_sources.begin(); it!=_sources.end(); ++it, ++i) {
      sources[i] = it->getSource();
      fds[i].fd = sources[i]->getfd();
      fds[i].revents = 0; // some platforms may not clear this in poll()
      fds[i].events = 0;
      if (it->getMask() & ReadableEvent) fds[i].events |= POLLIN_REQ;
      if (it->getMask() & WritableEvent) fds[i].events |= POLLOUT_REQ;
      if (it->getMask() & Exception) fds[i].events |= POLLEX_REQ;
    }

    // Check for events
    int nEvents = poll(&fds[0], source_cnt, (timeout_ms < 0) ? -1 : timeout_ms);

    if (nEvents < 0)
    {
#if defined(_WINDOWS)
      XmlRpcUtil::error("Error in XmlRpcDispatch::work: error in poll (%d).", WSAGetLastError());
#else
      if(errno != EINTR)
        XmlRpcUtil::error("Error in XmlRpcDispatch::work: error in poll (%d).", nEvents);
#endif
      _inWork = false;
      return;
    }

    // Process events
    for (i=0; i < source_cnt; ++i)
    {
      XmlRpcSource* src = sources[i];
      pollfd & pfd = fds[i];
      unsigned newMask = (unsigned) -1;
      // Only handle requested events to avoid being prematurely removed from dispatch
      bool readable = (pfd.events & POLLIN_REQ) == POLLIN_REQ;
      bool writable = (pfd.events & POLLOUT_REQ) == POLLOUT_REQ;
      bool oob = (pfd.events & POLLEX_REQ) == POLLEX_REQ;
      if (readable && (pfd.revents & POLLIN_CHK))
        newMask &= src->handleEvent(ReadableEvent);
      if (writable && (pfd.revents & POLLOUT_CHK))
        newMask &= src->handleEvent(WritableEvent);
      if (oob && (pfd.revents & POLLEX_CHK))
        newMask &= src->handleEvent(Exception);

      // Find the source iterator. It may have moved as a result of the way
      // that sources are removed and added in the call stack starting
      // from the handleEvent() calls above.
      SourceList::iterator thisIt;
      for (thisIt = _sources.begin(); thisIt != _sources.end(); thisIt++)
      {
        if(thisIt->getSource() == src)
          break;
      }
      if(thisIt == _sources.end())
      {
        XmlRpcUtil::error("Error in XmlRpcDispatch::work: couldn't find source iterator");
        continue;
      }

      if ( ! newMask) {
        _sources.erase(thisIt);  // Stop monitoring this one
        if ( ! src->getKeepOpen())
          src->close();
      } else if (newMask != (unsigned) -1) {
        thisIt->getMask() = newMask;
      }
    }

    // Check whether to clear all sources
    if (_doClear)
    {
      SourceList closeList = _sources;
      _sources.clear();
      for (SourceList::iterator it=closeList.begin(); it!=closeList.end(); ++it) {
	XmlRpcSource *src = it->getSource();
        src->close();
      }

      _doClear = false;
    }

    // Check whether end time has passed
    if (0 <= _endTime && getTime() > _endTime)
      break;
  }

  _inWork = false;
}


// Exit from work routine. Presumably this will be called from
// one of the source event handlers.
void
XmlRpcDispatch::exit()
{
  _endTime = 0.0;   // Return from work asap
}

// Clear all sources from the monitored sources list
void
XmlRpcDispatch::clear()
{
  if (_inWork)
    _doClear = true;  // Finish reporting current events before clearing
  else
  {
    SourceList closeList = _sources;
    _sources.clear();
    for (SourceList::iterator it=closeList.begin(); it!=closeList.end(); ++it)
      it->getSource()->close();
  }
}


double
XmlRpcDispatch::getTime()
{
#ifdef USE_FTIME
  struct timeb	tbuff;

  ftime(&tbuff);
  return ((double) tbuff.time + ((double)tbuff.millitm / 1000.0) +
	  ((double) tbuff.timezone * 60));
#else
  uint32_t sec, nsec;

  ros::ros_steadytime(sec, nsec);
	\
  return ((double)sec + (double)nsec / 1e9);
#endif /* USE_FTIME */
}


