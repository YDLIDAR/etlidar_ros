/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     example.cpp                                                     *
*  @brief    ETLidar Driver example                                          *
*  Details.                                                                  *
*                                                                            *
*  @author   Tony.Yang                                                       *
*  @email    chushuifurong618@eaibot.com                                     *
*  @version  1.0.0(版本号)                                                    *
*  @date     chushuifurong618@eaibot.com                                     *
*                                                                            *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2018/08/09 | 1.0.0     | Tony.Yang      | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/


#include <LIDAR/LIDARDevice.h>
#include <config.h>

#include <iostream>
#include <iterator>
#include <algorithm>
#include <thread>
#include <chrono>

#if defined(_WIN32)
# pragma warning(disable: 4786)
# pragma  comment(lib, "ETLidar.lib")
#endif

#define DEVICE_IP "192.168.0.11"

void ETLidarScanCallBack(const ydlidar::lidarData& data) {
    std::cout << "Receive data sample size:" << data.data.size() << std::endl;
    std::cout << "frame timestamp is:" << data.self_timestamp << std::endl;
    std::cout << "recive timestamp is:" << data.system_timestamp << std::endl;
}

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
        etlidar.RegisterLIDARDataCallback(&ETLidarScanCallBack);
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
