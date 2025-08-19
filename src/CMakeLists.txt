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

set(CONTROLLER_SRC
    Controller.cpp
    FootstepPlan.cpp
    ModelPredictiveControl.cpp
    PlanInterpolator.cpp
    ExternalPlanner.cpp
    Preview.cpp
    SwingFoot.cpp
    RobotUtils.cpp
    swing/SwingTrajCubicSplineSimple.cpp
    swing/SwingTrajLandingSearch.cpp)

set(CONTROLLER_HDR
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/Contact.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/Controller.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/FootstepPlan.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/ModelPredictiveControl.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/PlanInterpolator.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/ExternalPlanner.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/Preview.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/Sole.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/State.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/SwingFoot.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/utils/polynomials.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/utils/slerp.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/swing/SwingTrajCubicSplineSimple.h
    ${CMAKE_SOURCE_DIR}/include/lipm_walking/swing/SwingTrajLandingSearch.h)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED ${CONTROLLER_SRC} ${CONTROLLER_HDR})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC copra::copra eigen-quadprog::eigen-quadprog mc_rtc::mc_control_fsm
         mc_rtc::mc_planning ExternalFootstepPlanner_data)

install(TARGETS ${PROJECT_NAME} DESTINATION "${MC_RTC_LIBDIR}")

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME
                                                            "${PROJECT_NAME}")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

set(AROBASE "@")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../etc/LIPMWalkingSoftFoot.in.yaml"
               "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMWalkingSoftFoot.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMWalkingSoftFoot.yaml"
        DESTINATION "${MC_RTC_LIBDIR}/mc_controller/etc/")

add_subdirectory(states)

add_observer(RangeSensor observer/usbio.c observer/usbio.h observer/usbio_private.h observer/RangeSensor.cpp "${CMAKE_SOURCE_DIR}/include/lipm_walking/observer/RangeSensor.h")
target_link_libraries(RangeSensor PUBLIC mc_rtc::mc_control mc_mujoco::devices)
target_link_libraries(RangeSensor PRIVATE CppLinuxSerial::CppLinuxSerial)
target_link_libraries(RangeSensor PUBLIC Boost::filesystem)

add_library(CameraSensorShared OBJECT observer/CameraSensorShared.cpp "${CMAKE_SOURCE_DIR}/include/lipm_walking/observer/CameraSensorShared.h")
target_include_directories(CameraSensorShared PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)
target_link_libraries(CameraSensorShared PUBLIC mc_rtc::mc_control Boost::thread Boost::system)
set_target_properties(CameraSensorShared PROPERTIES POSITION_INDEPENDENT_CODE ON)
find_library(LIBRT rt)
if(LIBRT)
  target_link_libraries(CameraSensorShared PUBLIC ${LIBRT})
endif()

add_executable(CameraSensorResetShm observer/CameraSensorResetShm.cpp)
target_link_libraries(CameraSensorResetShm PUBLIC CameraSensorShared)

add_executable(CameraSensorServer observer/CameraSensorServer.cpp)
target_include_directories(CameraSensorServer PUBLIC "${cv_bridge_INCLUDE_DIRS}" "${sensor_msgs_INCLUDE_DIRS}" "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(CameraSensorServer PUBLIC CameraSensorShared Open3D::Open3D ceres ${realsense2_LIBRARY}
  image_transport cv_bridge mc_rtc::mc_rtc_ros "${OpenCV_LIBS}")

install(TARGETS CameraSensorResetShm CameraSensorServer DESTINATION bin)

add_observer(CameraSensor observer/CameraSensor.cpp "${CMAKE_SOURCE_DIR}/include/lipm_walking/observer/CameraSensor.h")
target_link_libraries(CameraSensor PUBLIC mc_rtc::mc_control mc_mujoco::devices CameraSensorShared)
target_include_directories(CameraSensor PUBLIC ${realsense2_DIR})
message("============================")
message(${realsense2_LIBRARY})
message(${realsense2_DIR})
message("============================")

install(
  TARGETS ${PROJECT_NAME} CppLinuxSerial qhull_r qhullcpp
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
