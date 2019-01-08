cmake_minimum_required(VERSION 2.8)
project( etlidar_driver )
set(LIDAR_VERSION_MAJOR 1)
set(LIDAR_VERSION_MINOR 0)
set(LIDAR_VERSION ${LIDAR_VERSION_MAJOR}.${LIDAR_VERSION_MINOR})
set( SDK_VERSION 1.0 )
#add_definitions(-std=c++11) # Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(include)
include_directories(src)

FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.h"
)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
add_subdirectory(samples)
add_library(etlidar_driver SHARED ${SDK_SRC})
IF (WIN32)
target_link_libraries(etlidar_driver ws2_32)
ELSE()
target_link_libraries(etlidar_driver rt pthread)
ENDIF()                    

########################################################
## Create configure file for inclusion in library
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h" )

set( GENERATED_HEADERS  ${CMAKE_CURRENT_BINARY_DIR}/config.h)

include_directories( ${CMAKE_BINARY_DIR} )
