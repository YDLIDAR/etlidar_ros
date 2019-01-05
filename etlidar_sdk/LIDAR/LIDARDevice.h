/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     LIDARDevice.h                                                   *
*  @brief    LIDAR Device Interface                                          *
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

#include "LIDARDriverInterface.h"
#include "Drivers/ETLidar/ETLidarDriver.h"
#include "DeviceException.h"

namespace ydlidar {

///////////////////////////////////////////////////////////////////////////////
// Generic LIDAR device
class LIDAR : public LIDARDriverInterface
{
    public:
        ///////////////////////////////////////////////////////////////
        explicit LIDAR(const std::string& default_ip = "192.168.0.11", int default_port = 9000) {
            ETLidarDriver* pDriver = new ETLidarDriver(default_ip, default_port);
            m_LIDAR  = std::shared_ptr<LIDARDriverInterface>( pDriver );
        }

        ///////////////////////////////////////////////////////////////
        ~LIDAR() {
            Clear();
        }

        ///////////////////////////////////////////////////////////////
        void Clear() {
            m_LIDAR = nullptr;
        }

        ///////////////////////////////////////////////////////////////
        void RegisterLIDARDataCallback(LIDARDriverDataCallback callback) {
            if( m_LIDAR ) {
                m_LIDAR->RegisterLIDARDataCallback( callback );
            } else {
                throw DeviceException("error: no driver initialized!");
            }
        }


protected:
    std::shared_ptr<LIDARDriverInterface>     m_LIDAR;

};

} /* namespace */
