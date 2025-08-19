# Copyright (c) 2018-2019, CNRS-UM LIRMM All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME lipm_walking_controller_softfoot)
set(PROJECT_DESCRIPTION
    "Walking controller based on linear inverted pendulum tracking")
set(PROJECT_URL https://github.com/stephane-caron/lipm_walking_controller/)
set(PROJECT_VERSION 1.6.0)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(INSTALL_DOCUMENTATION
    OFF
    CACHE BOOL "")
set(INSTALL_GENERATED_HEADERS
    OFF
    CACHE BOOL "" FORCE)
set(INSTALL_PKG_CONFIG_FILE
    OFF
    CACHE BOOL "" FORCE)
set(CXX_DISABLE_WERROR ON)

set(CATKIN_INSTALL_INTO_PREFIX_ROOT FALSE)

include(cmake/base.cmake)

project(${PROJECT_NAME} LANGUAGES C CXX)

add_project_dependency(copra REQUIRED)
add_project_dependency(Ceres REQUIRED)
add_project_dependency(Open3D REQUIRED)
find_package(realsense2 REQUIRED)
add_project_dependency(mc_rtc REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(
    FATAL_ERROR
      "mc_rtc must be built with ROS support for this controller to work")
endif()
add_project_dependency(eigen-quadprog REQUIRED)
# For Boost IPC
find_package(Boost REQUIRED COMPONENTS thread system)

if(EXISTS "${MC_RTC_LIBDIR}/mc_controller/etc/LIPMWalking.conf")
  message(
    FATAL_ERROR
      "\n \
############################################################ \n \
   Please uninstall the previous version (make uninstall)    \n \
############################################################ \n \
")
endif()

set(CATKIN_ENABLE_TESTING OFF)
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
add_project_dependency(trajectory_collection REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp variable_stiffness)
find_package(mc_mujoco REQUIRED)

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(QHULL_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(SERIAL_BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
add_subdirectory(3rd-party/qhull EXCLUDE_FROM_ALL)
add_subdirectory(3rd-party/CppLinuxSerial EXCLUDE_FROM_ALL)
add_subdirectory(3rd-party/serial EXCLUDE_FROM_ALL)

add_subdirectory(plugins/ExternalFootstepPlanner)
add_subdirectory(src)
add_subdirectory(nano)
add_subdirectory(camera)

set(PACKAGE_EXTRA_MACROS
    "set(LIPM_WALKING_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/lipm_walking_controller_softfoot/states\")
set(LIPM_WALKING_STATES_FILES \"\${LIPM_WALKING_STATES_LIBRARIES}/data\")")
