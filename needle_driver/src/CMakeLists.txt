cmake_minimum_required(VERSION 2.8)
project(simple)
SET( CMAKE_CXX_FLAGS "-std=c++11")

INCLUDE_DIRECTORIES(/home/charlie/Documents/kukaControl/NeedleDriver/)
LINK_DIRECTORIES(/home/charlie/Documents/kukaControl/NeedleDriver/)

INCLUDE_DIRECTORIES(/home/charlie/Documents/kukaControl/NeedleDriver/SewingMachine/)
LINK_DIRECTORIES(/home/charlie/Documents/kukaControl/NeedleDriver/SewingMachine/)

INCLUDE_DIRECTORIES(/home/charlie/Documents/kukaControl/NeedleDriver/qextserialport)
LINK_DIRECTORIES(/home/charlie/Documents/kukaControl/NeedleDriver/qextserialport)

SET(SIMPLE_PATH /home/charlie/Documents/kukaControl/NeedleDriver/)
SET(SIMPLE_HEADERS
${SIMPLE_PATH}/qextserialport/qextserialbase.h
${SIMPLE_PATH}/qextserialport/posix_qextserialport.h
${SIMPLE_PATH}/qextserialport/qextserialport.h

${SIMPLE_PATH}/SewingMachine/DynaMotorThread.h
${SIMPLE_PATH}/SewingMachine/dynamixel.h
${SIMPLE_PATH}/SewingMachine/dxl_hal.h

${SIMPLE_PATH}/NeedleDriverControl.h
${SIMPLE_PATH}/NeedleDriverInterface.h
)

SET(SIMPLE_SOURCES
${SIMPLE_PATH}/qextserialport/qextserialbase.cpp
${SIMPLE_PATH}/qextserialport/posix_qextserialport.cpp
${SIMPLE_PATH}/qextserialport/qextserialport.cpp

${SIMPLE_PATH}/SewingMachine/DynaMotorThread.cpp
${SIMPLE_PATH}/SewingMachine/dynamixel.cpp
${SIMPLE_PATH}/SewingMachine/dxl_hal.cpp

${SIMPLE_PATH}/NeedleDriverControl.cpp
${SIMPLE_PATH}/NeedleDriverInterface.cpp
${SIMPLE_PATH}/main.cpp
)

find_package( Qt4 REQUIRED )
include( ${QT_USE_FILE} )
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
INCLUDE_DIRECTORIES(
  ${QT_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
 )
#add_definitions( ${QT_DEFINITIONS} )
#include_directories( ${CMAKE_BINARY_DIR} )

QT4_WRAP_CPP( MOCSrcs ${SIMPLE_HEADERS}
)

SOURCE_GROUP("Generated" FILES
  ${MOCSrcs}
)


add_executable(simple ${SIMPLE_SOURCES} ${SIMPLE_HEADERS} ${MOCSrcs})

target_link_libraries(simple ${QT_LIBRARIES} )
