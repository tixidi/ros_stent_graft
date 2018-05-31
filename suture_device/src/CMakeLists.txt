cmake_minimum_required(VERSION 2.8.11)

project(SutureDevice)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)


# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Widgets SerialPort REQUIRED)



set( src main.cpp
        mainwindow.cpp
        Faulharbermotor.cpp
        footPadel.cpp)


set( headers 
	mainwindow.h
            Faulharbermotor.h
            footPadel.h
            yumi_suture_def.h)





file(GLOB UI_FILES *.ui)
qt5_wrap_ui(UISrcs ${UI_FILES})
# Tell CMake to create the helloworld executable
add_executable(SutureDevice ${src} ${headers} ${UISrcs})

# Use the Widgets module from Qt 5.
target_link_libraries(SutureDevice Qt5::Widgets Qt5::Core Qt5::SerialPort)
