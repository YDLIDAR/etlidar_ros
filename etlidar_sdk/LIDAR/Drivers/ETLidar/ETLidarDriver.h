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
#include <thread>

/* Header file to make it in image defined by HAL */
#include <LIDAR/LIDARDriverInterface.h>

/* Header files for socket variable */
#include <Sockets/PassiveSocket.h>
#include <Sockets/ActiveSocket.h>

namespace ydlidar {

class ETLidarDriver : public LIDARDriverInterface
{
public:
    /**
     * @brief ETLidarDriver
     * @param lidarIP
     * @param port
     */
    explicit ETLidarDriver(std::string lidarIP, int port=9000);

    ~ETLidarDriver();
    /**
     * @brief RegisterLIDARDataCallback
     * @param callback
     */
    void RegisterLIDARDataCallback(LIDARDriverDataCallback callback);

private:
    /**
     * @brief _ThreadFunc
     */
    void _ThreadFunc();

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
    * @brief Disconnect from ETLidar device.
    */
    void disconnect();

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
    * @brief Receive scan message.
    *
    * @param data pointer to lidarData buffer structure.
    */
    int getScanData(lidarData& data);

private:
    /* Variable for LIDAR compatibility */
    bool                      m_running;
    std::thread               m_callbackThread;
    LIDARDriverDataCallback   m_callback;

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
