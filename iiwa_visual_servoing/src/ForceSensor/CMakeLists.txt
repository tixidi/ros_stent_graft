cmake_minimum_required(VERSION 2.8)
project(simple)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
INCLUDE_DIRECTORIES ("/home/yang/Programs/ODV/API/LINUX_API_V1.5.1/include")

SET(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED )
find_package(aruco REQUIRED )

# header files
SET(HEADER_PATH /home/yang/Programs/ODV/API/LINUX_API_V1.5.1/include/omd/)

SET(simple_HEADER
${HEADER_PATH}/opto.h 
${HEADER_PATH}/optodaq.h
${HEADER_PATH}/optopackage.h
${HEADER_PATH}/optoports.h
${HEADER_PATH}/sensorconfig.h
)

# library
#find_library(simple_LIB ReflexxesTypeII HINTS /home/yang/Programs/ODV/API/LINUX_API_V1.5.1/lib)
LINK_DIRECTORIES(/home/yang/Programs/ODV/API/LINUX_API_V1.5.1/lib)

add_executable(simple main.cpp ${simple_HEADER})
target_link_libraries(simple -lOMD ${aruco_LIBS})
