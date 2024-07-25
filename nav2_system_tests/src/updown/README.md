# Nav2 Updown Test

This is a 'top level' system test which tests the lifecycle bringup and shutdown of the system. 

## To run the test
```
ros2 launch nav2_system_tests test_updown_launch.py
```

If the test passes, you should see this comment in the output:
```
[test_updown-13] [INFO] [test_updown]: ****************************************************  TEST PASSED!
```

To run the test in a loop 1000x, run the `test_updown_reliablity` script and log the output:
```
./test_updown_reliability |& tee /tmp/updown.log
```
When the test is completed, pipe the log to the `updownresults.py` script to get a summary of the results:
```
./updownresults.py < /tmp/updown.log`
```
