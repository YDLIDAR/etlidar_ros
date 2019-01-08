#/*****************************************************************************
#*  EAI TOF LIDAR DRIVER                                                      *
#*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
#*                                                                            *
#*  This file is part of EAI TOF LIDAR DRIVER.                                *
#*                                                                            *
#*  @file     CMakeLists.txt                                                  *
#*  @brief                                                                    *
#*  Details.                                                                  *
#*                                                                            *
#*  @author   Tony.Yang                                                       *
#*  @email    chushuifurong618@eaibot.com                                     *
#*  @version  1.0.0(版本号)                                                    *
#*  @date     chushuifurong618@eaibot.com                                     *
#*                                                                            *
#*                                                                            *
#*----------------------------------------------------------------------------*
#*  Remark         : Description                                              *
#*----------------------------------------------------------------------------*
#*  Change History :                                                          *
#*  <Date>     | <Version> | <Author>       | <Description>                   *
#*----------------------------------------------------------------------------*
#*  2018/08/09 | 1.0.0     | Tony.Yang      | Create file                     *
#*----------------------------------------------------------------------------*
#*                                                                            *
#*****************************************************************************/
cmake_minimum_required(VERSION 2.8)
PROJECT(etlidar_test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11

#Include directories
INCLUDE_DIRECTORIES(
    ${CMAKE_BINARY_DIR}
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/../
     ${CMAKE_CURRENT_BINARY_DIR}
)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
ADD_EXECUTABLE(${PROJECT_NAME} example.cpp)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} etlidar_driver)
LINK_DIRECTORIES(CMAKE_BINARY_DIR)

