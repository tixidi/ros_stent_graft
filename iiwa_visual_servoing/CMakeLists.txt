cmake_minimum_required(VERSION 2.8.3)
project(iiwa_visual_servoing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

#find cvblob
FIND_PACKAGE(cvBlob REQUIRED )

#find VXL
FIND_PACKAGE(VXL)
  IF(VXL_FOUND)
    INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)
  ELSE(VXL_FOUND)
    MESSAGE("VXL_DIR should be set to the VXL build directory.")
ENDIF(VXL_FOUND)

# Find VTK.
FIND_PACKAGE( VTK REQUIRED )
INCLUDE( ${VTK_USE_FILE} )
message(STATUS ${VTK_USE_FILE})

# use what QVTK built with
SET(QT_MOC_EXECUTABLE ${VTK_QT_MOC_EXECUTABLE} CACHE FILEPATH "")
SET(QT_UIC_EXECUTABLE ${VTK_QT_UIC_EXECUTABLE} CACHE FILEPATH "")
SET(QT_QMAKE_EXECUTABLE ${VTK_QT_QMAKE_EXECUTABLE} CACHE FILEPATH "")

# Find QT
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )

SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )


#OpenCV
SET(OpenCV_DIR "/usr/local/share/OpenCV")
FIND_PACKAGE( OpenCV 3.4 REQUIRED )
include_directories( "/usr/include/opencv")
#include_directories( "/usr/include/opencv2")

#aruco
INCLUDE("/usr/local/lib/cmake/Findaruco.cmake")
#find_package(aruco REQUIRED )

#include Erl
INCLUDE_DIRECTORIES ("src/iiwaControl/iiwa/Erl")
#include eigen
INCLUDE_DIRECTORIES (src/Eigen-3.3.3)
#INCLUDE_DIRECTORIES (/Eigen)

INCLUDE_DIRECTORIES ("src/iiwaControl/iiwa/asio/include")
INCLUDE_DIRECTORIES ("src/iiwaControl/iiwa/ReflexxesTypeII/include")

# iiwa
INCLUDE_DIRECTORIES ("src/iiwaControl/iiwa/IIWA_Sunrise/")
INCLUDE_DIRECTORIES ("src/iiwaControl/iiwa/PathPlanners/")

#include force sensor
INCLUDE_DIRECTORIES("src/ForceSensor/include")


INCLUDE_DIRECTORIES (
  ${VXL_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(
  ${QT_INCLUDE_DIR}
  ${QT_QTNETWORK_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
	src
 )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_visual_servoing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


###################the only thing need to change####################################################
# Set your files and resources here
SET( KukaControlSrcs 
#GUI
src/main.cpp
src/GUI/mainWindow.cpp
#kuka
src/KUKAControl/FRI/friremote.cpp
src/KUKAControl/FRI/friudp.cpp
src/Simulator/MandrelActor.cpp
src/Simulator/KukaGraphic.cpp
src/Simulator/manipulatorModel.cpp
src/KUKAControl/kukaKinematicModel.cpp
src/KUKAControl/kukaKinematicControl.cpp
src/KUKAControl/DualArmRobot.cpp
src/kukaMotionPlanning.cpp

#iiwa
src/iiwaControl/iiwaControl.cpp
#sunrise
src/iiwaControl/iiwa/IIWA_Sunrise/kukasunrise.cpp
src/iiwaControl/iiwa/PathPlanners/taskspaceplanner.cpp
src/iiwaControl/iiwa/PathPlanners/iiwaplanner.cpp
src/iiwaControl/iiwa/PathPlanners/kukasunriseplanned.cpp

#forcesensor
src/ForceSensor/forceSensor.cpp

#External input
#UDPCommunication/MasterSlaveComuniThread.cpp
#sewing machine sources
src/SewingMachine/dxl_hal.cpp
src/SewingMachine/dynamixel.cpp
src/SewingMachine/DynaMotorThread.cpp

#needle driver
src/NeedleDriver/NeedleDriverControl.cpp
src/NeedleDriver/NeedleDriverInterface.cpp
src/NeedleDriver/qextserialport/posix_qextserialport.cpp
src/NeedleDriver/qextserialport/qextserialport.cpp
src/NeedleDriver/qextserialport/qextserialbase.cpp
#Vision
src/VisionSystem/globals.cpp
src/VisionSystem/mainWindowVision.cpp
#VisionSystem/patterntracker/circlesgrid.hpp
src/VisionSystem/visualTrackingThread.cpp
src/VisionSystem/needleDrivertracking.cpp
src/VisionSystem/needleTracking.cpp

src/VisionSystem/handEyeAbsoluteOrientation/AbsoluteOrientation.cxx
src/VisionSystem/handEyeAbsoluteOrientation/Frame.cxx

#for needle tracking
src/VisionSystem/needleTracker/GWTracking.cpp
src/VisionSystem/needleTracker/mainFunctions.cpp

#for needle pose estimation 3D to 2D
src/VisionSystem/needlePoseEstimation3Dto2D/needlePoseEstimation.cpp

#for tool tracking
src/VisionSystem/tracktools/mycvdrawingutils.cpp
src/VisionSystem/tracktools/mymarkerdetector.cpp
src/VisionSystem/tracktools/tracktools.cpp

#for vision inspection
src/VisionSystem/mainWindowInspection.cpp
)

SET( KukaControlHeaders 
src/config.h
#GUI
src/GUI/mainWindow.h
#kuka
src/KUKAControl/FRI/friComm.h
src/KUKAControl/FRI/friremote.h
src/KUKAControl/FRI/friudp.h
src/Simulator/KukaGraphic.h
src/Simulator/MandrelActor.h
src/Simulator/manipulatorModel.h
src/KUKAControl/kukaKinematicModel.h
src/KUKAControl/kukaKinematicControl.h
src/KUKAControl/DualArmRobot.h
src/kukaMotionPlanning.h
src/KUKAControl/definitions.h
src/KUKAControl/functions.h

#sunrise
src/iiwaControl/iiwa/IIWA_Sunrise/datagramserver.h
src/iiwaControl/iiwa/IIWA_Sunrise/kukasunrise.h
src/iiwaControl/iiwa/PathPlanners/cartesianvelocityplanner.h
src/iiwaControl/iiwa/PathPlanners/iiwaplanner.h
src/iiwaControl/iiwa/PathPlanners/taskspaceplanner.h
src/iiwaControl/iiwa/PathPlanners/kukasunriseplanned.h

#iiwa
src/iiwaControl/iiwaControl.h

#forcesensor
src/ForceSensor/forceSensor.h

#UDPCommunication/MasterSlaveComuniThread.h
#sewing machine sources
src/SewingMachine/dxl_hal.h
src/SewingMachine/dynamixel.h
src/SewingMachine/DynaMotorThread.h

#neelde driver
src/NeedleDriver/NeedleDriverControl.h
src/NeedleDriver/NeedleDriverInterface.h
src/NeedleDriver/qextserialport/posix_qextserialport.h
src/NeedleDriver/qextserialport/qextserialport.h
src/NeedleDriver/qextserialport/qextserialbase.h

#Vision
src/VisionSystem/mainWindowVision.h
#VisionSystem/patterntracker/circlesgrid.hpp
#VisionSystem/patterntracker/circlesgrid.cpp
#VisionSystem/patterntracker/tracker.h
#VisionSystem/patterntracker/tracker.cpp
#VisionSystem/patterntracker/tracker_pattern.h
#VisionSystem/patterntracker/tracker_pattern.cpp
#VisionSystem/patterntracker/tracker_general.h
#VisionSystem/patterntracker/tracker_general.cpp

#Tool tracker
src/VisionSystem/tracktools/tracktools.h
src/VisionSystem/tracktools/mycvdrawingutils.h
src/VisionSystem/tracktools/mymarkerdetector.h

src/VisionSystem/globals.h
src/VisionSystem/visualTrackingThread.h
src/VisionSystem/needleDrivertracking.h
src/VisionSystem/needleTracking.h
src/VisionSystem/definitionsVision.h



#for point cloud matching
src/VisionSystem/handEyeAbsoluteOrientation/AbsoluteOrientation.h
src/VisionSystem/handEyeAbsoluteOrientation/Frame.h
src/VisionSystem/handEyeAbsoluteOrientation/Point3D.h

#for needle tracking
src/VisionSystem/needleTracker/GWTracking.h
src/VisionSystem/needleTracker/mainFunctions.h

#for needle pose estimation 3D to 2D
src/VisionSystem/needlePoseEstimation3Dto2D/needlePoseEstimation.h

#for vision inspection
src/VisionSystem/mainWindowInspection.h

)
####################################################################################################
QT4_WRAP_CPP( MOCSrcs ${KukaControlHeaders}
)

SOURCE_GROUP("Generated" FILES
  ${MOCSrcs}
)

LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/NeedleDriver/QserialPort/Lib)

# iiwa
find_library(REF_LIB ReflexxesTypeII
HINTS ../iiwa_test/src/IIWA/ReflexxesTypeII/Linux/x64/release/lib/shared)


# forcesensor
LINK_DIRECTORIES(src/ForceSensor/lib)

# source files
ADD_EXECUTABLE( iiwaVisualServoing
${KukaControlSrcs}
${KukaControlHeaders}
${MOCSrcs}
)

TARGET_LINK_LIBRARIES( iiwaVisualServoing
#${CMAKE_SOURCE_DIR}/VisionSystem/tracktools/libtracktools.a
${QT_LIBRARIES}
${QT_QTNETWORK_LIBRARY}
${OpenCV_LIBS}
${VTK_LIBRARIES}
${aruco_LIBS}
#-lReflexxesTypeII
QVTK
${CMAKE_SOURCE_DIR}/stentgraft_planning/iiwa_visual_servoing/src/VisionSystem/patterntracker/lib/libpatterntracker.a
vnl
vnl_algo
${cvBlob_LIBS}
${REF_LIB}
-lpthread
-lOMD
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/iiwa_visual_servoing.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/iiwa_visual_servoing_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_iiwa_visual_servoing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
