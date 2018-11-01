Message Filters --- chained message processing
==============================================

:mod:`message_filters` is a collection of message "filters" which take messages in,
either from a ROS subscription or another filter,
and may or may not output the message
at some time in the future, depending on a policy defined for that filter.

message_filters also defines a common interface for these filters, allowing you to chain them together.

The filters currently implemented in this package are:

 * :class:`message_filters.Subscriber` - A source filter, which wraps a ROS subscription.  Most filter chains will begin with a Subscriber.
 * :class:`message_filters.Cache` - Caches messages which pass through it, allowing later lookup by time stamp.
 * :class:`message_filters.TimeSynchronizer` - Synchronizes multiple messages by their timestamps, only passing them through when all have arrived.
 * :class:`message_filters.TimeSequencer` - Tries to pass messages through ordered by their timestamps, even if some arrive out of order.

Here's a simple example of using a Subscriber with a Cache::

    def myCallback(posemsg):
       print posemsg

    sub = message_filters.Subscriber("pose_topic", robot_msgs.msg.Pose)
    cache = message_filters.Cache(sub, 10)
    cache.registerCallback(myCallback)

The Subscriber here acts as the source of messages.  Each message is passed to the cache, which then passes it through to the
user's callback ``myCallback``.


Using the time synchronizer::

    from message_filters import TimeSynchronizer, Subscriber

    def gotimage(image, camerainfo):
        assert image.header.stamp == camerainfo.header.stamp
        print "got an Image and CameraInfo"

    tss = TimeSynchronizer(Subscriber("/wide_stereo/left/image_rect_color", sensor_msgs.msg.Image),
                           Subscriber("/wide_stereo/left/camera_info", sensor_msgs.msg.CameraInfo))
    tss.registerCallback(gotimage)


The message filter interface
----------------------------

For an object to be usable as a message filter, it needs to have one method,
``registerCallback``.  To collect messages from a message filter, register a callback with::

    anyfilter.registerCallback(my_callback)

The signature of ``my_callback`` varies according to the message filter.  For many filters it is simply::
  
    def my_callback(msg):

where ``msg`` is the message.

Message filters that accept input from an upstream
message filter (e.g. :class:`message_filters.Cache`) register their own
message handler as a callback.

Output connections are registered through the ``registerCallback()`` function.

.. automodule:: message_filters
    :members: Subscriber, Cache, TimeSynchronizer, TimeSequencer
    :inherited-members:

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

