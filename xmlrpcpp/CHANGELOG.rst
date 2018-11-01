^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xmlrpcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* fixes for OSX (`#1402 <https://github.com/ros/ros_comm/issues/1402>`_)
* take XmlRpcValue by *const* ref. in operator<< (`#1350 <https://github.com/ros/ros_comm/issues/1350>`_)
* fix various compiler warnings on bionic (`#1325 <https://github.com/ros/ros_comm/issues/1325>`_)

1.13.6 (2018-02-05)
-------------------
* fix xmlrpc timeout using monotonic clock (`#1249 <https://github.com/ros/ros_comm/issues/1249>`_)
* add tests and bug fixes for XmlRpcServer (`#1243 <https://github.com/ros/ros_comm/issues/1243>`_)
* add test and fix uninitialized data in XmlRpcClient (`#1244 <https://github.com/ros/ros_comm/issues/1244>`_)
* make xmlrpcpp specific include directory work in devel space (`#1261 <https://github.com/ros/ros_comm/issues/1261>`_)
* add base64 tests (`#1242 <https://github.com/ros/ros_comm/issues/1242>`_)
* add unit tests for XmlRpcDispatch (`#1232 <https://github.com/ros/ros_comm/issues/1232>`_)
* add unit tests and bug fixes for XmlRpcClient (`#1221 <https://github.com/ros/ros_comm/issues/1221>`_)

1.13.5 (2017-11-09)
-------------------
* add unit tests and bug fixes for XmlRpcSocket (`#1218 <https://github.com/ros/ros_comm/issues/1218>`_)
* add tests for XmlRpcValue and XML conversion (`#1216 <https://github.com/ros/ros_comm/issues/1216>`_)

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* fix problem when configuring tests without gtest being available (`#1197 <https://github.com/ros/ros_comm/issues/1197>`_)

1.13.2 (2017-08-15)
-------------------
* use poll() in favor of select() in the XmlRPCDispatcher (`#833 <https://github.com/ros/ros_comm/issues/833>`_)
* fix fall through warnings with g++ 7 (`#1139 <https://github.com/ros/ros_comm/issues/1139>`_)

1.13.1 (2017-07-27)
-------------------
* switch to libb64 for base64 encoding/decoding (`#1046 <https://github.com/ros/ros_comm/issues/1046>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* move headers to include/xmlrpcpp (`#962 <https://github.com/ros/ros_comm/issues/962>`_)

1.12.6 (2016-10-26)
-------------------

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------

1.12.2 (2016-06-03)
-------------------

1.12.1 (2016-04-18)
-------------------
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------

1.11.16 (2015-11-09)
--------------------

1.11.15 (2015-10-13)
--------------------

1.11.14 (2015-09-19)
--------------------

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------

1.11.10 (2014-12-22)
--------------------
* improve Android support (`#537 <https://github.com/ros/ros_comm/pull/537>`_)
* fix various defects reported by coverity

1.11.9 (2014-08-18)
-------------------

1.11.8 (2014-08-04)
-------------------

1.11.7 (2014-07-18)
-------------------

1.11.6 (2014-07-10)
-------------------

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* fix day comparison for rpc value of type timestamp (`#395 <https://github.com/ros/ros_comm/issues/395>`_)

1.11.0 (2014-03-04)
-------------------
* output error message when hostname lookup fails (`#364 <https://github.com/ros/ros_comm/issues/364>`_)

1.10.0 (2014-02-11)
-------------------

1.9.54 (2014-01-27)
-------------------

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* fix compilation and warnings with clang (`#291 <https://github.com/ros/ros_comm/issues/291>`_)

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------

1.9.44 (2013-03-21)
-------------------
* fix install destination for dll's under Windows

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* refine license description to LGPL-2.1

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
