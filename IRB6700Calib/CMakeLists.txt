﻿MESSAGE("---------------Eigen-------------------")
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
SET(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}) #needed for ceres for old version of cmake/FindEigen scripts
MESSAGE("Eigen3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}")
include_directories(include/)
file(GLOB SRC_PATH src/*.cpp)
file(GLOB HEADERS_PATH include/*.h)
add_executable (RobotCalib ${SRC_PATH} ${HEADERS_PATH})
TARGET_LINK_LIBRARIES(RobotCalib ${CERES_LIBRARIES})
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET RobotCalib PROPERTY CXX_STANDARD 20)
endif()