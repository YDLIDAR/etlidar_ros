/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ETLidarDriver.h                                                 *
*  @brief    TOF LIDAR DRIVER                                                *
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

#pragma once

/* Header file to enable threading and ergo callback */
#include "thread.h"
#include "locker.h"
#include "etlidar_protocol.h"
#include "PassiveSocket.h"
#include "SimpleSocket.h"
#include <vector>
/* Header files for socket variable */
#include "DeviceException.h"
#include "Console.h"

namespace ydlidar {

class ETLidarDriver
{
public:
    /**
     * @brief ETLidarDriver
     * @param lidarIP
     * @param port
     */
    explicit ETLidarDriver();

    ~ETLidarDriver();
    
    /**
     * @brief connect
     * @param ip_address
     * @param port
     * @return 
     */
      
    result_t connect(const std::string &ip_address, uint32_t port = 9000);

    /**
     * @brief isconnected
     * @return
     */
    bool isconnected() const;
    /**
    * @brief Disconnect from ETLidar device.
    */
    void disconnect();

    /**
     * @brief startScan
     * @param timeout
     * @return
     */
    result_t startScan( uint32_t timeout = DEFAULT_TIMEOUT) ;

    /**
     * @brief isscanning
     * @return
     */
    bool isscanning() const;

    /**
     * @brief stop
     * @return
     */
    result_t stop();
    
    /**
     * @brief grabScanData
     * @param scan
     * @param timeout
     * @return
     */
    result_t grabScanData(lidarData& scan, uint32_t timeout = DEFAULT_TIMEOUT) ;


private:
    /**
    * @brief Connect config port to ETLidar.
    * @param remote IP & port.
    */
    bool configPortConnect(const char* lidarIP, int tcpPort = 9000);
    /**
    * @brief Disconnect from ETLidar device.
    */
    char* configMessage(const char* descriptor, char* value = NULL);

    /**
    * @brief Start measurements.
    * After receiving this command ETLidar unit starts spinning laser and measuring.
    */
    bool startMeasure();

    /**
    * @brief Stop measurements.
    * After receiving this command ETLidar unit stop spinning laser and measuring.
    */
    bool stopMeasure();

    /**
    * @brief Get current scan configuration.
    * @returns scanCfg structure.
    */
    lidarConfig getScanCfg();

    /**
    * @brief Set scan configuration.
    * @param cfg structure containing scan configuration.
    */
    void setScanCfg(const lidarConfig& config);

    /**
    * @brief Connect data port to ETLidar.
    * @param remote IP & local port.
    */
    bool dataPortConnect(const char* lidarIP, int localPort = 8000);

    /**
     * @brief createThread
     * @return
     */
    result_t createThread();

    /**
     * @brief disableDataGrabbing
     */
    void disableDataGrabbing();
    /**
    * @brief Receive scan message.
    *
    * @param data pointer to lidarData buffer structure.
    */
    int getScanData(lidarData& data);

    /**
    * @brief parsing scan \n
    */
    int cacheScanData();
private:
    /* Variable for LIDAR compatibility */
    bool            isScanning;
    bool            isConnected;
    Event          	_dataEvent;			 ///<
    Locker         	_lock;				///<
    Thread 	       	_thread;				///<

    lidarData       global_scan_data;
    lidarConfig     m_config;
    size_t          offset_len;

    enum {
       DEFAULT_TIMEOUT 	= 2000,    /**< 默认超时时间. */
       DEFAULT_TIMEOUT_COUNT = 10,
     };


    /* ETLidar specific Variables */
    std::string               m_deviceIp;
    int                       m_port;
    int                       m_sampleRate;
    /* Sockets for ydlidar */
    CActiveSocket             socket_cmd;
    CPassiveSocket            socket_data;
    dataFrame                 frame;
    const char*               configValue[2] = {"0", "1"};

};

} /* namespace */
