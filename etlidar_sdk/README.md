
ETLIDAR SDK [![Build Status](https://travis-ci.org/cansik/etlidar_sdk.svg?branch=master)](https://travis-ci.org/cansik/etlidar_sdk) [![Build status](https://ci.appveyor.com/api/projects/status/2w9xm1dbafbi7xc0?svg=true)](https://ci.appveyor.com/project/cansik/etlidar_sdk) [![codebeat badge](https://codebeat.co/badges/3d8634b7-84eb-410c-b92b-24bf6875d8ef)](https://codebeat.co/projects/github-com-cansik-etlidar_sdk-master)
=====================================================================


Introduction
-------------------------------------------------------------------------------------------------------------------------------------------------------

ETLIDAR(https://www.ydlidar.com/) series is a set of high-performance and low-cost TOF LIDAR sensors, which is the perfect sensor of 2D SLAM, 3D reconstruction, multi-touch, and safety applications.

If you are using ROS (Robot Operating System), please use our open-source [ROS Driver]( https://github.com/ydlidar/etlidar_ros) .

Release Notes
-------------------------------------------------------------------------------------------------------------------------------------------------------
| Title      |  Version |  Data |
| :-------- | --------:|  :--: |
| SDK     |  1.0.1 |   2019-04-13  |

How to build ETLIDAR SDK samples
-------------------------------------------------------------------------------------------------------------------------------------------------------

    $ git clone https://github.com/ydlidar/etlidar_sdk

    $ cd etlidar_sdk

    $ git checkout master

    $ cd ..
    
Linux:

    $ mkdir build

    $ cd build

    $ cmake ../etlidar_sdk   ##windows: cmake -G "Visual Studio 14 2017 Win64" ../etlidar_sdk 

    $ make



Windows:
   $ mkdir build

    $ cd build

    $ cmake -G "Visual Studio 14 2017 Win64" ../etlidar_sdk 

    $ make


Compile wth Qt:
1). Qt configuration cmake
2). Open the CmakeLists.txt project file with Qt.
	
	
	
	
How to run ETLIDAR SDK samples
-------------------------------------------------------------------------------------------------------------------------------------------------------

linux:

    $ ./ydlidar_test
    Please enter the lidar IP[192.168.0.11](yes):yse
    [YDLidar]: SDK Version: 1.1
    [YDLidar]: LIDAR Version: 1.1
    [YDLidar]: Opening scan and checking whether Lidar is abnormal.........
    [YDLidar]: [YDLIDAR INFO] Now YDLIDAR is scanning ......

windows:

    $ ydlidar_test.exe
    Please enter the lidar IP[192.168.0.11](yes):yse
    [YDLidar]: SDK Version: 1.1
    [YDLidar]: LIDAR Version: 1.1
    [YDLidar]: Opening scan and checking whether Lidar is abnormal.........
    [YDLidar]: [YDLIDAR INFO] Now YDLIDAR is scanning ......


Data structure
-------------------------------------------------------------------------------------------------------------------------------------------------------

data structure:

	typedef struct _lidarPot {
		/**
		* @brief range
		*/
		float range;
		/**
		* @brief angle
		*/
		float angle;
		/**
		* @brief intensity
		*/
		int   intensity;
   	 
	}lidarPot;

	typedef struct _lidarData {
		/**
		* @brief ranges.
		*/
		std::vector<lidarPot> data;

		/**
		* @brief headFrameFlag.
		*/
		int headFrameFlag;

		/**
		* @brief frame timestamp in nanoseconds.
		*/
		uint64_t self_timestamp;

		/**
		* @brief system time.
		*/
		uint64_t system_timestamp;
		/**
		* @brief scan_time
		*/
		uint64_t scan_time;
	} lidarData;

example parsing:

    for(size_t i =0; i < scan.data.size(); i++) {

      // current angle
      float angle = scan.data[i].angle ;// radian format

      //current distance
      float angle = scan.data[i].range ;//meters

      //current intensity
      int intensity = scan.data[i].intensity;

    }

laser callback function code :


        

Quick Start
-----------

The best way to learn how to use sdk is to follow the tutorials in our
sdk guide:

https://github.com/yangfuyuan/etlidar_sdk/Samples

If you want to learn from code examples, take a look at the examples in the
[Samples](Samples) directory.


### Include Header
	#include "ETLidarDriver.h"
	#include <config.h>
	#define DEVICE_IP "192.168.0.11"

### Simple Usage

```c++

int main(int argc, char **argv) {

    char* lidarIp = DEVICE_IP;
	if (argc > 1) {
        lidarIp = argv[1];
    }
    ydlidar::init(argc, argv);

     ydlidar::ETLidarDriver lidar;
     result_t ans = lidar.connect(lidarIp);
     if(!IS_OK(ans)) {
     	ydlidar::console.error("Failed to connecting lidar...");
     	return 0;
     }
     
     bool rs = lidar.turnOn();
     while (rs && ydlidar::ok()) {
    	lidarData scan;
    	ans = lidar.grabScanData(scan);
    	if(IS_OK(ans)) {
      		ydlidar::console.message("scan recevied[%llu]: %d ranges",scan.system_timestamp, scan.data.size());
    	} else {
      		ydlidar::console.warning("Failed to get scan data");
    	}
     }
    lidar.turnOff();
    lidar.disconnect();
    return 0;
    
}
```

Note: Use sdk to be a "try catch" syntax to do exception handling.


Contact EAI
---------------

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)

