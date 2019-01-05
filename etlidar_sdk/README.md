
ETLIDAR SDK [![Build Status](https://travis-ci.org/cansik/etlidar_sdk.svg?branch=master)](https://travis-ci.org/cansik/etlidar_sdk) [![Build status](https://ci.appveyor.com/api/projects/status/2w9xm1dbafbi7xc0?svg=true)](https://ci.appveyor.com/project/cansik/etlidar_sdk) [![codebeat badge](https://codebeat.co/badges/3d8634b7-84eb-410c-b92b-24bf6875d8ef)](https://codebeat.co/projects/github-com-cansik-etlidar_sdk-master)
=====================================================================


Introduction
-------------------------------------------------------------------------------------------------------------------------------------------------------

ETLIDAR(https://www.ydlidar.com/) series is a set of high-performance and low-cost TOF LIDAR sensors, which is the perfect sensor of 2D SLAM, 3D reconstruction, multi-touch, and safety applications.

If you are using ROS (Robot Operating System), please use our open-source [ROS Driver]( https://github.com/yangfuyuan/etlidar_ros) .

Release Notes
-------------------------------------------------------------------------------------------------------------------------------------------------------
| Title      |  Version |  Data |
| :-------- | --------:|  :--: |
| SDK     |  1.0.0 |   2018-8-14  |

How to build ETLIDAR SDK samples
-------------------------------------------------------------------------------------------------------------------------------------------------------

    $ git clone https://github.com/yangfuyuan/etlidar_sdk

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
windows:

    $ ydlidar_test.exe

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
	#include <LIDAR/LIDARDevice.h>
	#include <config.h>
	
	#include <thread>
	#include <chrono>
	
	#define DEVICE_IP "192.168.0.11"

### Callback

```c++
      void LaserScanCallback(const ydlidar::lidarData& scan) {

          std::cout<< "received scan size: "<< scan.data.size()<<std::endl;

    	  std::cout<< "scan   system time: "<< scan.system_timestamp<<std::endl;

    	  std::cout<< "scan     self time: "<< scan.self_timestamp<<std::endl;

         for(size_t i =0; i < scan.data.size(); i++) {

      		// current angle
      		float angle = scan.data[i].angle ;// radian format

     		//current distance
    		float angle = scan.data[i].range ;//meters

      		//current intensity
      		int intensity = scan.data[i].intensity;

          }

        }
```

### Simple Usage

```c++

int main(int argc, char **argv) {

    char* lidarIp = DEVICE_IP;
	if (argc > 1) {
        lidarIp = argv[1];
    }
    
    ydlidar::init(argc, argv);


    std::cout <<"SDK Version: "<< SDK_VERSION <<std::endl;
    std::cout <<"LIDAR Version: "<< EHLIDAR_VERSION <<std::endl;
   try {

        ydlidar::LIDAR etlidar(lidarIp);
        etlidar.RegisterLIDARDataCallback(&LaserScanCallback);
        while (ydlidar::ok()) {
            std::this_thread::sleep_for (std::chrono::milliseconds(50));
        }

    }catch(ydlidar::DeviceException& e) {
        std::cout << e.what()<< std::endl;
    }catch(...) {
        std::cout <<"Unkown error" << std::endl;
    }
    
    
return 0;
    
}
```

Note: Use sdk to be a "try catch" syntax to do exception handling.


Contact EAI
---------------

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)

