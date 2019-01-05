/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     LIDARDriverInterface.h                                          *
*  @brief    LIDAR  Interface                                                *
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

#include <functional>
#include "Utils.h"


namespace ydlidar {

typedef std::function<void (ydlidar::lidarData&)> LIDARDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
/// Generic LIDAR driver interface
class LIDARDriverInterface
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual ~LIDARDriverInterface() {}
        virtual void RegisterLIDARDataCallback(LIDARDriverDataCallback callback) = 0;
};

} /* namespace */
