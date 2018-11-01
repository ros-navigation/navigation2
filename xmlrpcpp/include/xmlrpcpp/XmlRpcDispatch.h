
#ifndef _XMLRPCDISPATCH_H_
#define _XMLRPCDISPATCH_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#include "xmlrpcpp/XmlRpcDecl.h"

#ifndef MAKEDEPEND
# include <list>
# include <vector>
#endif

namespace XmlRpc {

  // An RPC source represents a file descriptor to monitor
  class XmlRpcSource;

  //! An object which monitors file descriptors for events and performs
  //! callbacks when interesting events happen.
  class XMLRPCPP_DECL XmlRpcDispatch {
  public:
    //! Constructor
    XmlRpcDispatch();
    ~XmlRpcDispatch();

    //! Values indicating the type of events a source is interested in
    enum EventType {
      ReadableEvent = 1,    //!< data available to read
      WritableEvent = 2,    //!< connected/data can be written without blocking
      Exception     = 4     //!< out-of-band data has arrived
    };
    
    //! Monitor this source for the event types specified by the event mask
    //! and call its event handler when any of the events occur.
    //!  @param source The source to monitor
    //!  @param eventMask Which event types to watch for. \see EventType
    void addSource(XmlRpcSource* source, unsigned eventMask);

    //! Stop monitoring this source.
    //!  @param source The source to stop monitoring
    void removeSource(XmlRpcSource* source);

    //! Modify the types of events to watch for on this source
    void setSourceEvents(XmlRpcSource* source, unsigned eventMask);


    //! Watch current set of sources and process events for the specified
    //! duration (in ms, -1 implies wait forever, or until exit is called)
    void work(double msTime);

    //! Exit from work routine
    void exit();

    //! Clear all sources from the monitored sources list. Sources are closed.
    void clear();

    // helper returning current steady/monotonic time
    double getTime();

    // A source to monitor and what to monitor it for
    struct MonitoredSource {
      MonitoredSource(XmlRpcSource* src, unsigned mask) : _src(src), _mask(mask) {}
      XmlRpcSource* getSource() const { return _src; }
      unsigned& getMask() { return _mask; }
      XmlRpcSource* _src;
      unsigned _mask;
    };

    // A list of sources to monitor
    typedef std::list< MonitoredSource > SourceList; 

    // Sources being monitored
    SourceList _sources;
  protected:

    // When work should stop (-1 implies wait forever, or until exit is called)
    double _endTime;

    bool _doClear;
    bool _inWork;

  };
} // namespace XmlRpc

#endif  // _XMLRPCDISPATCH_H_
