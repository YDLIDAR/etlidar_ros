ETLIDAR ROS PACKAGE V1.0.1
=====================================================================

ROS node and test application for ETLIDAR

Visit EAI Website for more details about ETLIDAR.

How to build ETLIDAR ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build etlidar_node and etlidar_client

How to run ETLIDAR ros package
=====================================================================
There're two ways to run ETLIDAR ros package

1. Run ETLIDAR node and view in the rviz
------------------------------------------------------------
roslaunch etlidar_ros lidar_view.launch

You should see ETLIDAR's scan result in the rviz.

2. Run ETLIDAR node and view using test application
------------------------------------------------------------
roslaunch etlidar_ros lidar.launch

rosrun etlidar_ros etlidar_client

You should see ETLIDAR's scan result in the console


Parameters
=====================================================================
ip (string, default: 192.168.0.11)

    2d tof lidar ip.

frame_id (string, default: laser_frame)

    frame ID for the device.

angle_min (double, default: -150)

    Min valid angle (°) for LIDAR data.

angle_max (double, default: 150)

    Max valid angle (°) for LIDAR data.

range_min (double, default: 0.035)

    Min valid range (m) for LIDAR data.

range_max (double, default: 64.0)

    Max valid range (m) for LIDAR data.

ignore_array (string, default: "")

    Set the current angle range value to zero.




Upgrade Log
=====================================================================

2018-12-5 version:1.0.1

   1.fix angle resolution error

2018-12-5 version:1.0.0

   1.2D tof Lidar ros package


