cmake_minimum_required(VERSION 2.8.3)
project(etlidar_ros)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-std=c++11) # Use C++11

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

#add_subdirectory(sdk)

set(SDK_PATH "./etlidar_sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/*.cpp"
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SDK_PATH}
  ${SDK_PATH}/include
  ${SDK_PATH}/src
)

add_executable(etlidar_node src/etlidar_node.cpp  ${SDK_SRC})
add_executable(etlidar_client src/etlidar_client.cpp)

target_link_libraries(etlidar_node
   ${catkin_LIBRARIES} 
 )
target_link_libraries(etlidar_client
   ${catkin_LIBRARIES} 
 )
########################################################
## Create configure file for inclusion in library
configure_file("${SDK_PATH}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h" )

set( GENERATED_HEADERS  ${CMAKE_CURRENT_BINARY_DIR}/config.h)

include_directories( ${CMAKE_BINARY_DIR} )


install(TARGETS etlidar_node etlidar_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch startup
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


