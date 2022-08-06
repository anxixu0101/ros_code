# ros_code
一些关于ros/C++从入门，进阶，到修仙的代码,部分代码也是为了以后的激光/视觉SLAM服务。代码多采用C++，虽然我水平有限，但我会尽量让自己的代码保持优雅。不定时更新


## msg_communication
ros发送/接收话题代码，接收话题时调用了回调函数，应用了多线程同时运行

## grid_map
简单的发布nav_msgs::OccupancyGrid数据格式的栅格地图，可以在rviz下订阅/map话题并显示观看


------------------------------------

# ros_code
Some about ros/C++ from entry, advanced, to the fairy code, part of the code is also for the future laser/visual SLAM service. Code mostly uses C++, although my level is limited, but I will try to keep their own code elegant. Irregular update

## msg_communication
Ros sends/receives the topic code. When receiving the topic, the callback function is called, and multiple threads are applied simultaneously

