# publish_test
Project publish_test for ROS2

This is a testing tool to investigate the impact of the number of publishes on the CPU utilization of a process.
It is divided into publisher/subscriber processes, and controls the number of publications with specified parameters.

### install
```
cd ~/ros2_perf/publish_test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=Off -DCMAKE_CXX_FLAGS="-w"
```

### parameters  
The “base part” is the parameter of the part to which the steady-state load is applied.
Normally, the “base part” parameter is fixed, and the “addtional part” parameter is varied by changing the topic_count to measure the effect on CPU time of an increase or decrease in the number of publications.

#### publisher process side
|No|content|base part name|additional part name|default|
|-|-|-|-|-|
|1|publish counts|topic_count|var_topic_count|200|
|2|publish unit|unit|var_unit|10|
|3|frequency|frequency|var_frequency|10|
|4|message size|msg_size|var_msg_size|100|
|5|qos depth|qos_depth|var_qos_depth|10|
|6|debug output suppressed|output_suppressed|output_suppressed|false|

#### subscriber process side
|No|content|base part name|additional part name|default|
|-|-|-|-|-|
|1|publish counts|topic_count|var_topic_count|200|
|2|frequency|frequency|var_frequency|10|
|3|qos depth|qos_depth|var_qos_depth|10|
|4|debug output suppressed|output_suppressed|output_suppressed|false|


### run
When the following command is executed, the publisher and subscriber processes are launched and start communicating with the specified parameters.
```
cd ~/ros2_perf/publish_test
. install/local_setup.bash

ros2 launch publish_test publish_test.launch.xml topic_count:=100 unit:=10 var_topic_count:=100 var_unit:=10 msg_size:=128 var_msg_size:=128  output_suppressed:=true
```

### stop
To stop, press CTRL+C. The number of publications and messages received at this point will be displayed, which can be used as a reference to check the communication time and number of messages, lost messages, etc. However, please keep in mind that the count at the time of SIGINT capture is displayed.

```
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[publisher_node-1] [INFO 1726711093.770826005] [LOG]:
[publisher_node-1] ================ 1589295 (signal_pub_handler() at /home/akm/vtune/publish_test/src/main.cpp:19)
[publisher_node-1] [INFO 1726711093.770884056] [LOG]:
[publisher_node-1] === PUB: base ===
[publisher_node-1]   TX=34349 *1 (signal_pub_handler() at /home/akm/vtune/publish_test/src/main.cpp:22)
[publisher_node-1] [INFO 1726711093.770895019] [LOG]:
[publisher_node-1] === PUB: var ===
[publisher_node-1]   TX=34339 *2 (signal_pub_handler() at /home/akm/vtune/publish_test/src/main.cpp:25)
[subscriber_node-2] [INFO 1726711093.770849246] [LOG]:
[subscriber_node-2] ++++++++++++++++ 1589297 (signal_sub_handler() at /home/akm/vtune/publish_test/src/main.cpp:33)
[subscriber_node-2] [INFO 1726711093.770928191] [LOG]:
[subscriber_node-2] === SUB: base ===
[subscriber_node-2]   RX=34349 *3 (signal_sub_handler() at /home/akm/vtune/publish_test/src/main.cpp:36)
[subscriber_node-2] [INFO 1726711093.770950176] [LOG]:
[subscriber_node-2] === SUB: var ===
[subscriber_node-2]   RX=34339 *4 (signal_sub_handler() at /home/akm/vtune/publish_test/src/main.cpp:39)
[INFO] [subscriber_node-2]: process has finished cleanly [pid 1589297]
[INFO] [publisher_node-1]: process has finished cleanly [pid 1589295]

*1 Number of messages published in “base part"  
*2 Number of messages published in “addtional part"  
*3 Number of messages subscribed to by “base part"  
*4 Number of messages subscribed by “addtional part"  
```
