RPLIDAR ROS package
=====================================================================

ROS 2 node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

How to build rplidar ros package
=====================================================================
1) Clone this project to your colcon's workspace src folder

git clone https://github.com/ncnynl/rplidar_ros2 rplidar_ros 

2) Running `colcon build` to build `rplidar_node`


How to run rplidar ros package
=====================================================================

```
$ ros2 run rplidar_ros rplidar_node  # for RPLIDAR A1/A2
$ rviz2
```

RPLidar frame
=====================================================================
RPLidar frame must be broadcasted according to pictures shown below

![](./rplidar_A1.png)

![](./rplidar_A2.png)
