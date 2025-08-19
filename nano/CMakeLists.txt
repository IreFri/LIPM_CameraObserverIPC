add_executable(test_0 test_0.cpp)
target_link_libraries(test_0 PRIVATE CppLinuxSerial::CppLinuxSerial)

add_executable(test_1 test_1.cpp)
target_link_libraries(test_1 PRIVATE Boost::filesystem mc_rtc::mc_rtc_gui mc_rtc::mc_control)

# add_executable(test_1_map test_1_map.cpp)
# target_link_libraries(test_1_map PRIVATE Boost::filesystem mc_rtc::mc_rtc_gui mc_rtc::mc_control)

# add_executable(test_1_vertical test_1_vertical.cpp)
# target_link_libraries(test_1_vertical PRIVATE Boost::filesystem mc_rtc::mc_rtc_gui mc_rtc::mc_control)

add_executable(test_2 test_2.cpp)

set(TEST_3_SRC
  test_3.cpp
  serial.cc
  serial.h
  list_ports_linux.cc
  unix.cc
  unix.h
)

add_executable(test_3 ${TEST_3_SRC})

add_executable(calibration calibration.cpp)
target_link_libraries(calibration PRIVATE ceres mc_rtc::mc_control)