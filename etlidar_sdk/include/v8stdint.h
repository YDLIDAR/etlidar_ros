#ifndef V8STDINT_H_
#define V8STDINT_H_

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <signal.h>
#include <cerrno>
#include <stdexcept>
#include <csignal>
#include <sys/stat.h>
#if defined(_MSC_VER)
#include <io.h>
#endif

#if !defined(_MSC_VER)
#include <unistd.h>
#endif

#define UNUSED(x) (void)x

#if !defined(_MSC_VER)
#	define _access access
#endif

#if defined(_WIN32) && !defined(__MINGW32__)
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
typedef int int32_t;
typedef unsigned int uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else

#include <stdint.h>

#endif

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


#ifdef _AVR_
typedef uint8_t        _size_t;
#define THREAD_PROC
#elif defined (WIN64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef uint32_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (_M_X64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _size_t;
#define THREAD_PROC
#elif defined (__ICCARM__)
typedef uint32_t       _size_t;
#define THREAD_PROC
#endif

typedef _size_t (THREAD_PROC *thread_proc_t)(void *);

typedef int32_t result_t;
typedef uint64_t TTimeStamp;

#define RESULT_OK      0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL    -2

#define INVALID_TIMESTAMP (0)

enum {
  DEVICE_DRIVER_TYPE_SERIALPORT = 0x0,
  DEVICE_DRIVER_TYPE_TCP = 0x1,
};


#define IS_OK(x)    ( (x) == RESULT_OK )
#define IS_TIMEOUT(x)  ( (x) == RESULT_TIMEOUT )
#define IS_FAIL(x)  ( (x) == RESULT_FAIL )


#if defined(_WIN32)
#include <windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <sys/time.h>
#include <unistd.h>

static inline void delay(uint32_t ms) {
  while (ms >= 1000) {
    usleep(1000 * 1000);
    ms -= 1000;
  };

  if (ms != 0) {
    usleep(ms * 1000);
  }
}
#endif


namespace ydlidar {

void init(int argc, char *argv[]);
bool ok();
void shutdownNow();
bool fileExists(const std::string filename);

}


#endif  // V8STDINT_H_
