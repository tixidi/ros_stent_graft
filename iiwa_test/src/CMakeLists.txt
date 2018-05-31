cmake_minimum_required(VERSION 2.8)
project(iiwaTets)

SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET( CMAKE_CXX_FLAGS "-std=c++11")

include_directories ("${PROJECT_SOURCE_DIR}")

#include Erl
INCLUDE_DIRECTORIES ("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/Erl")
INCLUDE_DIRECTORIES ("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/Eigen-3.3.3")
INCLUDE_DIRECTORIES ("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/asio/include")
INCLUDE_DIRECTORIES ("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/ReflexxesTypeII/include")

# iiwa
INCLUDE_DIRECTORIES ("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/IIWA_Sunrise/")
INCLUDE_DIRECTORIES ("/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/")

# cpps
SET(IIWA_SRC
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/IIWA_Sunrise/kukasunrise.cpp
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/taskspaceplanner.cpp
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/iiwaplanner.cpp
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/kukasunriseplanned.cpp
)

# header files
SET(IIWA_HEADER
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/IIWA_Sunrise/kukasunrise.h
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/IIWA_Sunrise/datagramserver.h
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/cartesianvelocityplanner.h
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/taskspaceplanner.h
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/iiwaplanner.h
/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/PathPlanners/kukasunriseplanned.h
)

# library
find_library(REF_LIB ReflexxesTypeII HINTS /home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwaTest/src/ros_ws/src/stentgraft_planning/iiwaTest/src/IIWA/ReflexxesTypeII/Linux/x64/release/lib/shared)

add_executable(iiwaTets main.cpp ${IIWA_SRC} ${IIWA_HEADER})
target_link_libraries(iiwaTets ${REF_LIB} -lpthread)
