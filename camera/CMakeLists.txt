# add_executable(test_0_camera test_0.cpp)
# target_link_libraries(test_0_camera PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control realsense2)

# find_package(OpenGL REQUIRED)
# add_executable(test_1_camera test_1.cpp)
# target_link_libraries(test_1_camera PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control realsense2 glfw ${OPENGL_LIBRARIES})

add_executable(re-export_log re_export_log.cpp)
target_link_libraries(re-export_log PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control mc_mujoco::devices)

find_package(Open3D REQUIRED)

add_executable(ground_estimation_2 ground_estimation_2.cpp)
target_link_libraries(ground_estimation_2 PRIVATE mc_rtc::mc_control mc_mujoco::devices Open3D::Open3D ceres)
