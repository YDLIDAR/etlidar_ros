/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     Utils.h                                                         *
*  @brief    Driver data type                                                *
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

#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <string.h>
#include <signal.h>
#include <cerrno>
#include <stdexcept>
#include <csignal>
#if defined(_MSC_VER)
#include <io.h>
#endif

#if !defined(_MSC_VER)
#include <unistd.h>
#endif

#if defined(_WIN32)
#if defined(ETLidar_STATIC)
    #define ETLidar_EXPORT
#elif defined(ETLidar_EXPORTS)
    #define ETLidar_EXPORT __declspec(dllexport)
#else
    #define ETLidar_EXPORT __declspec(dllimport)
#endif

#else 

#include <stdint.h>
#define ETLidar_EXPORT
#define _itoa(value, str, radix) {sprintf(str, "%d", value);}

#endif // ifdef WIN32


#define UNUSED(x) (void)x

#if !defined(_MSC_VER)
#	define _access access
#endif

#define valName(val) (#val)
#define valLastName(val) \
{ \
    char* strToken; \
    char str[64]; \
    strncpy(str, (const char*)val, sizeof(str)); \
    strToken = strtok(str, "."); \
    while (strToken != NULL) { \
        strcpy(val, (const char*)strToken); \
        strToken = strtok(NULL, "."); \
    } \
}


/**
 * @class dataFrame
 * @brief data frame Structure.
 *
 * @author jzhang
 */
#define FRAME_PREAMBLE 0xFFEE
#define LIDAR_2D 0x2
#define DATA_FRAME 0x1
#define DEFAULT_INTENSITY 10
#define DSL(c, i) ((c << i) & (0xFF << i))


#define DEFAULT_CONNECT_TIMEOUT_SEC 2
#define DEFAULT_CONNECT_TIMEOUT_USEC 0


namespace ydlidar {


typedef struct _dataFrame {
    uint16_t frameHead;
    uint8_t deviceType;
    uint8_t frameType;
    uint8_t dataIndex;
    uint8_t frameIndex;
    uint32_t timestamp;
    uint8_t headFrameFlag;
    uint8_t dataFormat;
    uint8_t disScale;
    uint32_t startAngle;
    uint32_t dataNum;
    uint32_t frameCrc;
    char frameBuf[2048];
} dataFrame;

/**
 * @class lidarConfig
 * @brief Structure containing scan configuration.
 *
 * @author jzhang
 */
typedef struct _lidarConfig {
    /**
     * @brief Scanning enable.
     */
    int laser_en;

    /**
     * @brief rotate enable.
     */
    int motor_en;

    /**
     * @brief motor RPM.
     */
    int motor_rpm;

    /**
     * @brief start FOV angle.
     */
    int fov_start;

    /**
     * @brief end FOV angle.
     */
    int fov_end;

    /**
     * @brief data receive interface, USB or Ethernet.
     */
    int trans_sel;

    /**
     * @brief data receive IP.
     */
    char dataRecvIp[16];

    /**
     * @brief data receive PORT.
     */
    int dataRecvPort;

    /**
     * @brief device network config, HDCP or Manual.
     */
    int dhcp_en;

    /**
     * @brief device IP.
     */
    char deviceIp[16];

    /**
     * @brief device netmask.
     */
    char deviceNetmask[16];

    /**
     * @brief device gateway ip.
     */
    char deviceGatewayIp[16];
} lidarConfig;



/**
 * @class lidarPot
 * @brief Structure containing single scan point.
 *
 * @author Tony.Yang
 */
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

/**
 * @class lidarData
 * @brief Structure containing single scan message.
 *
 * @author jzhang
 */
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

}

// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define HAS_SIGACTION
#endif


static volatile sig_atomic_t g_signal_status = 0;

#ifdef HAS_SIGACTION
static struct sigaction old_action;
#else
typedef void (* signal_handler_t)(int);
static signal_handler_t old_signal_handler = 0;
#endif

#ifdef HAS_SIGACTION
inline struct sigaction
set_sigaction(int signal_value, const struct sigaction & action)
#else
inline signal_handler_t
set_signal_handler(int signal_value, signal_handler_t signal_handler)
#endif
{
#ifdef HAS_SIGACTION
  struct sigaction old_action;
  ssize_t ret = sigaction(signal_value, &action, &old_action);
  if (ret == -1)
#else
  signal_handler_t old_signal_handler = std::signal(signal_value, signal_handler);
  // NOLINTNEXTLINE(readability/braces)
  if (old_signal_handler == SIG_ERR)
#endif
  {
    const size_t error_length = 1024;
    // NOLINTNEXTLINE(runtime/arrays)
    char error_string[error_length];
#ifndef _WIN32
#if (defined(_GNU_SOURCE) && !defined(ANDROID) &&(_POSIX_C_SOURCE >= 200112L))
    char * msg = strerror_r(errno, error_string, error_length);
    if (msg != error_string) {
      strncpy(error_string, msg, error_length);
      msg[error_length - 1] = '\0';
    }
#else
    int error_status = strerror_r(errno, error_string, error_length);
    if (error_status != 0) {
      throw std::runtime_error("Failed to get error string for errno: " + std::to_string(errno));
    }
#endif
#else
    strerror_s(error_string, error_length, errno);
#endif
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::runtime_error(
      std::string("Failed to set SIGINT signal handler: (" + std::to_string(errno) + ")") +
      error_string);
    // *INDENT-ON*
  }

#ifdef HAS_SIGACTION
  return old_action;
#else
  return old_signal_handler;
#endif
}

inline void trigger_interrupt_guard_condition(int signal_value) {
    g_signal_status = signal_value;
    signal(signal_value, SIG_DFL);
}

inline void
#ifdef HAS_SIGACTION
signal_handler(int signal_value, siginfo_t * siginfo, void * context)
#else
signal_handler(int signal_value)
#endif
{
  // TODO(wjwwood): remove? move to console logging at some point?
  printf("signal_handler(%d)\n", signal_value);

#ifdef HAS_SIGACTION
  if (old_action.sa_flags & SA_SIGINFO) {
    if (old_action.sa_sigaction != NULL) {
      old_action.sa_sigaction(signal_value, siginfo, context);
    }
  } else {
    if (
      old_action.sa_handler != NULL &&  // Is set
      old_action.sa_handler != SIG_DFL &&  // Is not default
      old_action.sa_handler != SIG_IGN)  // Is not ignored
    {
      old_action.sa_handler(signal_value);
    }
  }
#else
  if (old_signal_handler) {
    old_signal_handler(signal_value);
  }
#endif

  trigger_interrupt_guard_condition(signal_value);
}

namespace ydlidar {

inline void init(int argc, char *argv[]) {
    UNUSED(argc);
    UNUSED(argv);
#ifdef HAS_SIGACTION
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  sigemptyset(&action.sa_mask);
  action.sa_sigaction = ::signal_handler;
  action.sa_flags = SA_SIGINFO;
  ::old_action = set_sigaction(SIGINT, action);
  set_sigaction(SIGTERM, action);

#else
  ::old_signal_handler = set_signal_handler(SIGINT, ::signal_handler);
  // Register an on_shutdown hook to restore the old signal handler.
#endif
}
inline bool ok() {
  return g_signal_status == 0;
}
inline void shutdown() {
  trigger_interrupt_guard_condition(SIGINT);
}

inline bool fileExists(const std::string filename) {
    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
}



}

