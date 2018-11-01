# ROS2 Message Filters 

`ros2_message_filters` blends various messages based on the conditions that filter needs to met and derives from ROS2 porting of [ROS message_filters](http://wiki.ros.org/message_filters). It collects commonly used message "filtering" algorithms into a common space. A message filter is defined as something which a message arrives into and may or may not be spit back out of at a later point in time. 


**The API is a combination of parts:**
- Filter Pattern
  - `registerCallback(cb)`
  - `connectInput(filter)`
- Subscriber
  - `subscribe(nodep, topic, 1, myCallback)`
- Time Synchronizer
  - Input : 
    - C++: Up to 9 separate filters, each of which is of the form `void callback(const std::shared_ptr<M const>&)`. The number of filters supported is determined by the number of template arguments the class was created with. 
    - Python: N separate filters, each of which has signature `callback(msg)`. 
  - Output : 
    - C++: For message types M0..M8, `void callback(const std::shared_ptr<M0 const>&`, ..., `const std::shared_ptr<M8 const>&)`. The number of parameters is determined by the number of template arguments the class was created with.
    - Python: `callback(msg0.. msgN)`. The number of parameters is determined by the number of template arguments the class was created with.

```
  #include <message_filters/subscriber.h>
  #include <message_filters/time_synchronizer.h>
  #include <sensor_msgs/Image.h>
  #include <sensor_msgs/CameraInfo.h>

  using namespace sensor_msgs;
  using namespace message_filters;
    
  void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
  {
     // Solve all of perception here...
  }
  
  int main(int argc, char** argv)
  {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("test_node");
    Subscriber<Image> image_sub(nh.get(), "image");
    Subscriber<CameraInfo> info_sub(nh.get(), "camera");
    TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
    sync.registerCallback(std::bind(&callback, _1, _2));
    rclcpp::spin(nh);
    return 0;
   }
```

**Environment**

* Hardware: Compliant to ROS2 HW requirements
* Software: ROS2 + Supported OS(Linux, Win, Mac)

**How to build and test ROS2 message filters**
- Clone and build ROS2 source code under ros2 workspace, please refer to [ROS2 Installation](http://github.com/ros2/ros2/wiki/Installation) for more details and the following is the example in Ubuntu 16.04 LTS.
```
$cd ~/<ros2_workspace>
$src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
```
- Build message filters
```
$cd src/ros2
$git clone https://github.com/intel/ros2_message_filters.git
$source <ros2_workspace>/install/local_setup.bash
$ament build src/ros2/ros2_message_filters
```
- Run unit tests
```
$ament test src/ros2/ros2_message_filters

1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '/home/jwang-robot/ros2_ws/src/ros2/message_filters':
1:  - /home/jwang-robot/ros2_ws/build/message_filters/message_filters-test_simple --gtest_output=xml:/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_simple.gtest.xml
1: [==========] Running 2 tests from 1 test case.
1: [----------] Global test environment set-up.
1: [----------] 2 tests from SimpleFilter
1: [ RUN      ] SimpleFilter.callbackTypes
1: [       OK ] SimpleFilter.callbackTypes (0 ms)
1: [ RUN      ] SimpleFilter.oldRegisterWithNewFilter
1: [       OK ] SimpleFilter.oldRegisterWithNewFilter (0 ms)
1: [----------] 2 tests from SimpleFilter (0 ms total)
1: 
1: [----------] Global test environment tear-down
1: [==========] 2 tests from 1 test case ran. (0 ms total)
1: [  PASSED  ] 2 tests.
1: -- run_test.py: return code 0
1: -- run_test.py: inject classname prefix into gtest result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_simple.gtest.xml'
1: -- run_test.py: verify result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_simple.gtest.xml'
 1/10 Test  #1: message_filters-test_simple ....................   Passed    0.09 sec
test 2
      Start  2: message_filters-msg_cache_unittest

2: Test timeout computed to be: 60
2: -- run_test.py: invoking following command in '/home/jwang-robot/ros2_ws/src/ros2/message_filters':
2:  - /home/jwang-robot/ros2_ws/build/message_filters/message_filters-msg_cache_unittest --gtest_output=xml:/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-msg_cache_unittest.gtest.xml
2: [==========] Running 5 tests from 1 test case.
2: [----------] Global test environment set-up.
2: [----------] 5 tests from Cache
2: [ RUN      ] Cache.easyInterval
2: [       OK ] Cache.easyInterval (0 ms)
2: [ RUN      ] Cache.easySurroundingInterval
2: [       OK ] Cache.easySurroundingInterval (0 ms)
2: [ RUN      ] Cache.easyUnsorted
2: [       OK ] Cache.easyUnsorted (0 ms)
2: [ RUN      ] Cache.easyElemBeforeAfter
2: [       OK ] Cache.easyElemBeforeAfter (0 ms)
2: [ RUN      ] Cache.eventInEventOut
2: [       OK ] Cache.eventInEventOut (0 ms)
2: [----------] 5 tests from Cache (0 ms total)
2: 
2: [----------] Global test environment tear-down
2: [==========] 5 tests from 1 test case ran. (0 ms total)
2: [  PASSED  ] 5 tests.
2: -- run_test.py: return code 0
2: -- run_test.py: inject classname prefix into gtest result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-msg_cache_unittest.gtest.xml'
2: -- run_test.py: verify result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-msg_cache_unittest.gtest.xml'
 2/10 Test  #2: message_filters-msg_cache_unittest .............   Passed    0.09 sec
test 3
      Start  3: message_filters-test_chain

3: Test timeout computed to be: 60
3: -- run_test.py: invoking following command in '/home/jwang-robot/ros2_ws/src/ros2/message_filters':
3:  - /home/jwang-robot/ros2_ws/build/message_filters/message_filters-test_chain --gtest_output=xml:/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_chain.gtest.xml
3: [==========] Running 8 tests from 1 test case.
3: [----------] Global test environment set-up.
3: [----------] 8 tests from Chain
3: [ RUN      ] Chain.simple
3: [       OK ] Chain.simple (0 ms)
3: [ RUN      ] Chain.multipleFilters
3: [       OK ] Chain.multipleFilters (0 ms)
3: [ RUN      ] Chain.addingFilters
3: [       OK ] Chain.addingFilters (1 ms)
3: [ RUN      ] Chain.inputFilter
3: [       OK ] Chain.inputFilter (0 ms)
3: [ RUN      ] Chain.nonSharedPtrFilter
3: [       OK ] Chain.nonSharedPtrFilter (0 ms)
3: [ RUN      ] Chain.retrieveFilter
3: [       OK ] Chain.retrieveFilter (0 ms)
3: [ RUN      ] Chain.retrieveFilterThroughBaseClass
3: [       OK ] Chain.retrieveFilterThroughBaseClass (0 ms)
3: [ RUN      ] Chain.retrieveBaseClass
3: [       OK ] Chain.retrieveBaseClass (0 ms)
3: [----------] 8 tests from Chain (1 ms total)
3: 
3: [----------] Global test environment tear-down
3: [==========] 8 tests from 1 test case ran. (1 ms total)
3: [  PASSED  ] 8 tests.
3: -- run_test.py: return code 0
3: -- run_test.py: inject classname prefix into gtest result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_chain.gtest.xml'
3: -- run_test.py: verify result file '/home/jwang-robot/ros2_ws/build/message_filters/test_results/message_filters/message_filters-test_chain.gtest.xml'
 3/10 Test  #3: message_filters-test_chain .....................   Passed    0.09 sec
test 4
...
...
...
100% tests passed, 0 tests failed out of 10

Label Time Summary:
gtest     =  16.15 sec (9 tests)
pytest    =   0.54 sec (1 test)

Total Test time (real) =  16.69 sec

```

- Run fuzz tests (`disabled by default`)
```
$./build/message_filters/message_filters-test_fuzz

[==========] Running 3 tests from 3 test cases.
[----------] Global test environment set-up.
[----------] 1 test from TimeSequencer
[ RUN      ] TimeSequencer.fuzz_sequencer
[       OK ] TimeSequencer.fuzz_sequencer (5118 ms)
[----------] 1 test from TimeSequencer (5118 ms total)

[----------] 1 test from TimeSynchronizer
[ RUN      ] TimeSynchronizer.fuzz_synchronizer
[       OK ] TimeSynchronizer.fuzz_synchronizer (5013 ms)
[----------] 1 test from TimeSynchronizer (5013 ms total)

[----------] 1 test from Subscriber
[ RUN      ] Subscriber.fuzz_subscriber
[       OK ] Subscriber.fuzz_subscriber (5010 ms)
[----------] 1 test from Subscriber (5010 ms total)

[----------] Global test environment tear-down
[==========] 3 tests from 3 test cases ran. (15142 ms total)
[  PASSED  ] 3 tests.
```

## Known issue
* python not support headless message
* Not verify with Windows and OS X environment and there may be some errors

## Security check
If there is any security issue, it should be reported using process at https://01.org/security.
