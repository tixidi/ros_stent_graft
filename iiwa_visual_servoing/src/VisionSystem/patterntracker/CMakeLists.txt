cmake_minimum_required(VERSION 2.8.11)

project(patterntracker)

# OpenCV
find_package( OpenCV 2.4 REQUIRED )

if(WIN32)
    # Postfix of .lib and .dll
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

file (GLOB TRACKER_LIB_SRC "*.cpp")
file (GLOB TRACKER_LIB_HEADER "*.h")

add_library (patterntracker ${TRACKER_LIB_SRC} ${TRACKER_LIB_HEADER})
#add_library (patterntracker tracker.cpp tracker.h)
