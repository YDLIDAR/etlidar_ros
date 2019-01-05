/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ETLidarDriver.cpp                                               *
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

#include "ETLidarDriver.h"
#include <stdio.h>

/*Socket Specific headers */
#include <unistd.h>
#include <errno.h>

/* Headers from Exception */
#include <DeviceException.h>

using namespace ydlidar;


/////////////////////////////////////////////////////////////////////////////////////////
// port defaults to 9000 if not provided.
ETLidarDriver::ETLidarDriver(std::string lidarIP, int port) :
    m_running(false),
    m_callback(nullptr),
    m_deviceIp(lidarIP),
    m_port(port),
    m_sampleRate(20000)
{

    socket_data.SetSocketType(CSimpleSocket::SocketTypeUdp);
    socket_cmd.SetConnectTimeout(DEFAULT_CONNECT_TIMEOUT_SEC, DEFAULT_CONNECT_TIMEOUT_USEC);

    if(!configPortConnect(m_deviceIp.c_str(), m_port)) {
        throw DeviceException(socket_cmd.DescribeError());
    }

    lidarConfig config = getScanCfg();
    //config.laser_en = 1;
    //setScanCfg(config);

    if(!dataPortConnect(config.deviceIp, config.dataRecvPort)) {
        throw DeviceException(socket_data.DescribeError());
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void ETLidarDriver::_ThreadFunc()
{
    lidarData scandata;
    int timeout = 0;
    startMeasure();
    printf("[ETLIDAR INFO] Now ETLIDAR is scanning ......\n");
    while (m_running) {
        try {
            lidarData local_data;
            getScanData(local_data);
            if(local_data.headFrameFlag) {
                if(m_callback&&scandata.data.size()) {
                  m_callback(scandata);
                }
                scandata.data.clear();
                scandata = local_data;
                scandata.system_timestamp = local_data.system_timestamp - 1e9/m_sampleRate*local_data.data.size();
            } else {
                scandata.scan_time += local_data.scan_time;
                scandata.data.insert( scandata.data.end(), local_data.data.begin(),  local_data.data.end());
            }
            timeout = 0;
        }catch ( TimeoutException& e) {
            timeout++;
            if(timeout > 5) {
                m_running = false;
                throw DeviceException(e.what());
            } else {
                continue;
            }

        } catch (CorruptedDataException& e) {
            fprintf(stderr, "scan data parse error: %s\n", e.what());
            fflush(stderr);
            continue;
        }catch (DeviceException& e) {
            m_running = false;
            throw DeviceException(e.what());
        } catch (...) {
            m_running = false;
            throw DeviceException("Unkown error......");
        }

    }
    stopMeasure();
}


bool ETLidarDriver::configPortConnect(const char* lidarIP, int tcpPort) {

    if(!socket_cmd.IsSocketValid()) {
        if(!socket_cmd.Initialize()) {
            return false;
        }
    }
    socket_cmd.SetNonblocking();
    if(!socket_cmd.Open(lidarIP, tcpPort)) {
        return false;
    }
    socket_cmd.SetBlocking();
    return socket_cmd.IsSocketValid();
}

void ETLidarDriver::disconnect() {
    m_running = false;
    if(socket_cmd.IsSocketValid()) {
        stopMeasure();
    }
    socket_cmd.Close();
    socket_data.Close();
}


char* ETLidarDriver::configMessage(const char* descriptor, char* value) {
    char buf[100];
    char transDesc[32];
    char recvDesc[32];
    static char recvValue[32];

    strncpy(transDesc, descriptor, sizeof(transDesc));
    valLastName(transDesc);

    sprintf(buf, "%s=%s\n", transDesc, value);
    //printf("TX command:%s", buf);
    //fflush(stdout);

    socket_cmd.Send(reinterpret_cast<uint8_t*>(buf),strlen(buf) );

    memset(buf, 0, sizeof(buf));
    if(socket_cmd.Select(0, 50000)) {
        socket_cmd.Receive(sizeof(buf),reinterpret_cast<uint8_t*>(buf));
        //printf("RX command:%s\n", buf);
        //fflush(stdout);
        if (2 == sscanf(buf, "%[^=]=%[^=]", recvDesc, recvValue)) {
            if (!strcmp(transDesc, recvDesc)) {
                return recvValue;
            } else {
                return NULL;
            }
        } else {
            return NULL;
        }

    } else {
        return value;
    }
    return NULL;
}

bool ETLidarDriver::startMeasure() {
    bool ret ;
    ret = configMessage("motor_en", (char *)configValue[1]) != NULL;
    ret &= configMessage("laser_en", (char *)configValue[1]) != NULL;
    return ret;
}

bool ETLidarDriver::stopMeasure() {
    bool ret ;
    ret = configMessage("motor_en", (char *)configValue[0]) != NULL;
    ret &= configMessage("laser_en", (char *)configValue[0]) != NULL;
    return ret;
}

lidarConfig ETLidarDriver::getScanCfg() {
    lidarConfig cfg;

    char* result = configMessage(valName(cfg.laser_en));
    if( result != NULL) {
        cfg.laser_en = atoi(result);
    }

    result = configMessage(valName(cfg.motor_en));
    if( result != NULL) {
        cfg.motor_en = atoi(result);
    }

    result = configMessage(valName(cfg.motor_rpm));
    if( result != NULL) {
        cfg.motor_rpm = atoi(result);
    }

    result = configMessage(valName(cfg.fov_start));
    if( result != NULL) {
        cfg.fov_start = atoi(result);
    }

    result = configMessage(valName(cfg.fov_end));
    if( result != NULL) {
        cfg.fov_end = atoi(result);
    }

    result = configMessage(valName(cfg.trans_sel));
    if( result != NULL) {
        cfg.trans_sel = atoi(result);
    }

    result = configMessage(valName(cfg.dataRecvPort));
    if( result != NULL) {
        cfg.dataRecvPort = atoi(result);
    }


    result = configMessage(valName(cfg.dhcp_en));
    if( result != NULL) {
        cfg.dhcp_en = atoi(result);
    }

    result = configMessage(valName(cfg.dataRecvIp));
    if( result != NULL) {
        strcpy(cfg.dataRecvIp, result);
    }

    result = configMessage(valName(cfg.deviceIp));
    if( result != NULL) {
        strcpy(cfg.deviceIp, result);
    }

    result = configMessage(valName(cfg.deviceNetmask));
    if( result != NULL) {
        strcpy(cfg.deviceNetmask, result);
    }


    result = configMessage(valName(cfg.deviceGatewayIp));
    if( result != NULL) {
        strcpy(cfg.deviceGatewayIp, result);
    }

    return cfg;
}


void ETLidarDriver::setScanCfg(const lidarConfig& config) {
    char str[32];

    _itoa(config.laser_en, str, 10);
    configMessage(valName(config.laser_en), str);

    _itoa(config.motor_en, str, 10);
    configMessage(valName(config.motor_en), str);

    _itoa(config.motor_rpm, str, 10);
    configMessage(valName(config.motor_rpm), str);

    _itoa(config.fov_start, str, 10);
    configMessage(valName(config.fov_start), str);

    _itoa(config.fov_end, str, 10);
    configMessage(valName(config.fov_end), str);

    _itoa(config.trans_sel, str, 10);
    configMessage(valName(config.trans_sel), str);

    _itoa(config.dataRecvPort, str, 10);
    configMessage(valName(config.dataRecvPort), str);

    _itoa(config.dhcp_en, str, 10);
    configMessage(valName(config.dhcp_en), str);

    configMessage(valName(config.dataRecvIp), (char *)config.dataRecvIp);
    configMessage(valName(config.deviceIp), (char *)config.deviceIp);
    configMessage(valName(config.deviceNetmask), (char *)config.deviceNetmask);
    configMessage(valName(config.deviceGatewayIp), (char *)config.deviceGatewayIp);
}



bool ETLidarDriver::dataPortConnect(const char* lidarIP, int localPort) {
    if(!socket_data.IsSocketValid() ) {
        if(socket_data.Initialize()) {
            if(!socket_data.Listen(NULL, localPort)) {
                socket_data.Close();
                return false;
            }
            socket_data.SetReceiveTimeout(5,0);
        }
    }
    if(lidarIP) {
        //printf("ydlidar device ip: %s\n", lidarIP);
        //fflush(stdout);
    }
    return socket_data.IsSocketValid();
}

int ETLidarDriver::getScanData(lidarData& data) {

    int offset;
    /* wait data from socket. */
    if( socket_data.Receive(  sizeof(frame.frameBuf), reinterpret_cast<uint8_t*>(frame.frameBuf)) < 0) {
        throw TimeoutException(socket_data.DescribeError());
    }

    data.system_timestamp = CStatTimer::GetCurrentTime();
    /* check frame head */
    frame.frameHead = DSL(frame.frameBuf[0], 8) | DSL(frame.frameBuf[1], 0);
    if (FRAME_PREAMBLE != frame.frameHead) {
        throw CorruptedDataException("recv data error for header");
    }

    /* check device type */
    frame.deviceType = (frame.frameBuf[2] >> 4) & 0xf;
    if (LIDAR_2D != frame.deviceType) {
        throw CorruptedDataException("recv data error for device type");
    }

    /* check frame type */
    frame.frameType = frame.frameBuf[2] & 0xf;
    if (DATA_FRAME != frame.frameType) {
        throw CorruptedDataException("recv data error for frame type");
    }

    /* parser head length */
    frame.dataIndex = (frame.frameBuf[3] >> 4) & 0xf;
    frame.dataIndex = (frame.dataIndex +1)*4;

    /* parser frame index */
    frame.frameIndex = frame.frameBuf[3] & 0xf;

    /* parser timestamp */
    frame.timestamp = DSL(frame.frameBuf[4], 24) | DSL(frame.frameBuf[5], 16)
                    | DSL(frame.frameBuf[6], 8)  | DSL(frame.frameBuf[7], 0);

    /* parser head frame flag */
    frame.headFrameFlag = (frame.frameBuf[8] >> 4) & 0xf;

    /* parser data format */
    frame.dataFormat = frame.frameBuf[8] & 0xf;

    /* parser distance scale */
    frame.disScale = frame.frameBuf[9];

    /* parser start angle */
    frame.startAngle = DSL(frame.frameBuf[10], 8) | DSL(frame.frameBuf[11], 0);

    /* parser valid data number */
    frame.dataNum = DSL(frame.frameBuf[12], 8) | DSL(frame.frameBuf[13], 0);

    /* parser frame crc */
    frame.frameCrc = DSL(frame.frameBuf[14], 8) | DSL(frame.frameBuf[15], 0);

    /* parser data */
    data.data.resize(frame.dataNum);
    lidarPot pot;

    if (frame.dataFormat == 0) {
      for (uint32_t i = 0; i<frame.dataNum; i++) {
          offset = frame.dataIndex + 4*i;
          pot.intensity = (uint16_t)(DSL(frame.frameBuf[offset], 8) | DSL(frame.frameBuf[offset+1], 0));
          pot.range = (float)(DSL(frame.frameBuf[offset+2], 8) | DSL(frame.frameBuf[offset+3], 0))/1000.f;
          if (i > 0) {
              pot.angle = (float)(frame.frameCrc - frame.startAngle)/(frame.dataNum - 1)/100.f + data.data[i-1].angle;
              if (pot.angle >= 360)
                  pot.angle -= 360;
          }else {
              pot.angle = (float)frame.startAngle/100.f;
          }
          data.data[i] = pot;

      }

    } else {
      for (unsigned int i = 0; i<frame.dataNum; i++) {
          offset = frame.dataIndex + 4*i;
          pot.intensity = (uint16_t)frame.frameBuf[offset];
          pot.range = (float)(DSL(frame.frameBuf[offset+2], 8) | DSL(frame.frameBuf[offset+3], 0))/1000.f;
          if (i > 0) {
              pot.angle = (float)(frame.frameBuf[offset+1])/100.f + data.data[i-1].angle;
              if (pot.angle >= 360)
                  pot.angle -= 360;
          }else {
              pot.angle = (float)frame.startAngle/100.f;
          }
          data.data[i] = pot;
      }
    }
    data.self_timestamp = (uint64_t)(frame.timestamp*100);
    data.headFrameFlag = (int)frame.headFrameFlag;
    data.scan_time     = 1e9/m_sampleRate*frame.dataNum;

    return frame.dataNum;

}

/////////////////////////////////////////////////////////////////////////////////////////
ETLidarDriver::~ETLidarDriver()
{
    m_running = false;
    if( m_callbackThread.joinable() ) {
        m_callbackThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void ETLidarDriver::RegisterLIDARDataCallback(LIDARDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &ETLidarDriver::_ThreadFunc, this );
}
